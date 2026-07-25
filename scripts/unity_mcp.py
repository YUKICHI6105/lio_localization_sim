#!/usr/bin/env python3
"""Call a Unity MCP tool from the shell.

The official Unity MCP relay speaks JSON-RPC over stdio and connects to the running Editor
through a named pipe, so a plain subprocess is enough to drive it.  This exists so the stage-4
verification loop can run without waiting for a Claude Code session restart to pick up a
newly registered MCP server.

Usage:
    unity_mcp.py <tool_name> [json_arguments]
    unity_mcp.py --list

Examples:
    unity_mcp.py --list
    unity_mcp.py Unity_GetConsoleLogs '{"count": 50}'
    unity_mcp.py Unity_RunCommand '{"script": "Debug.Log(\"hi\");"}'
"""
import json
import subprocess
import sys
import time

RELAY = "/mnt/c/Users/kouza.FUKU-PC/.unity/relay/relay_win.exe"
# The relay has to start bun, attach to the Editor's named pipe and enumerate tools before it
# can serve a call; measured cold start is a few seconds, so allow generous headroom.
CONNECT_WAIT = 6.0
CALL_TIMEOUT = 180


def rpc(messages, want_id, timeout=CALL_TIMEOUT):
    """Send JSON-RPC messages to the relay and collect replies until `want_id` arrives.

    stdin must stay open while waiting: the relay attaches to the Editor's named pipe
    asynchronously and shuts down as soon as it sees EOF, so writing everything up front and
    closing (as subprocess.run does) kills it before it has discovered any tools.
    """
    import queue
    import threading

    proc = subprocess.Popen(
        [RELAY, "--mcp"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, bufsize=1)

    lines: "queue.Queue[str]" = queue.Queue()

    def reader():
        for line in proc.stdout:
            lines.put(line)
        lines.put("")  # sentinel on EOF

    threading.Thread(target=reader, daemon=True).start()

    try:
        for i, message in enumerate(messages):
            proc.stdin.write(json.dumps(message) + "\n")
            proc.stdin.flush()
            # Give the relay time to reach Unity before the call that needs the connection.
            time.sleep(CONNECT_WAIT if i == 0 else 0.5)

        replies = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = lines.get(timeout=1.0)
            except queue.Empty:
                continue
            if line == "":
                break
            line = line.strip()
            if not line:
                continue
            try:
                reply = json.loads(line)
            except json.JSONDecodeError:
                continue
            replies.append(reply)
            if reply.get("id") == want_id:
                break
    finally:
        try:
            proc.stdin.close()
        except OSError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

    stderr = proc.stderr.read() if proc.stderr else ""
    return replies, stderr


def handshake():
    return [
        {"jsonrpc": "2.0", "id": 1, "method": "initialize",
         "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                    "clientInfo": {"name": "claude-code", "version": "1.0.0"}}},
        {"jsonrpc": "2.0", "method": "notifications/initialized"},
    ]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    if sys.argv[1] == "--list":
        replies, err = rpc(handshake() + [
            {"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}}], want_id=2)
        for r in replies:
            for t in (r.get("result") or {}).get("tools", []) or []:
                print(t["name"])
        if not replies:
            print("NO REPLY. relay stderr:\n" + err, file=sys.stderr)
            return 1
        return 0

    tool = sys.argv[1]
    args = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    replies, err = rpc(handshake() + [
        {"jsonrpc": "2.0", "id": 2, "method": "tools/call",
         "params": {"name": tool, "arguments": args}}], want_id=2)

    for r in replies:
        if r.get("id") != 2:
            continue
        if "error" in r:
            print("ERROR: " + json.dumps(r["error"], ensure_ascii=False, indent=1))
            return 1
        result = r.get("result", {})
        # MCP returns content blocks; print text verbatim so callers can parse it.
        for block in result.get("content", []) or []:
            if block.get("type") == "text":
                print(block.get("text", ""))
            else:
                print(f"[{block.get('type')} block omitted]")
        if result.get("isError"):
            return 1
        return 0

    print("NO RESULT for the call. relay stderr:\n" + err, file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
