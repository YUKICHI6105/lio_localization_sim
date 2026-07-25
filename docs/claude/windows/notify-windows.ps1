param(
    [string]$Title,
    [string]$Message,
    [string]$Cwd,
    [string]$SessionId,
    [switch]$Relaunched
)

$ErrorActionPreference = 'SilentlyContinue'

function Write-DiagLog([string]$msg) {
    try {
        $line = (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') + " [PID=$PID relaunched=$($Relaunched.IsPresent)] " + $msg
        Add-Content -Path "$HOME\.claude\notify-diagnostic.log" -Value $line -Encoding utf8
    } catch {}
}
Write-DiagLog 'script started'

# ProcessStartInfo.ArgumentList は .NET Core 2.1 以降にしか存在せず、
# WSLから呼ばれる powershell.exe (Windows PowerShell 5.1 / .NET Framework) では null になる。
# そのため文字列の .Arguments を使う必要があり、Win32のコマンドライン規則で自前クォートする。
function ConvertTo-Win32Arg([string]$s) {
    if ($null -eq $s -or $s -eq '') { return '""' }
    if ($s -notmatch '[\s"]') { return $s }
    $out = '"'
    $bs = 0
    foreach ($ch in $s.ToCharArray()) {
        if ($ch -eq '\') { $bs++; continue }
        if ($ch -eq '"') { $out += ('\' * ($bs * 2 + 1)) + '"' }
        else            { $out += ('\' * $bs) + $ch }
        $bs = 0
    }
    return $out + ('\' * ($bs * 2)) + '"'
}

# Windows TerminalがデフォルトのターミナルアプリになっているとStart-Process -WindowStyle Hidden や
# CreateNoWindowが効かず、コンソールウィンドウが表示されたまま残ることがある。
# そのため呼ばれた直後に自分自身のコンソールウィンドウを直接非表示にする(最も確実)。
try {
    Add-Type -Name ClaudeNotifyConsoleWindow -Namespace ClaudeNotify -MemberDefinition @'
[System.Runtime.InteropServices.DllImport("kernel32.dll")]
public static extern System.IntPtr GetConsoleWindow();
[System.Runtime.InteropServices.DllImport("user32.dll")]
public static extern bool ShowWindow(System.IntPtr hWnd, int nCmdShow);
'@ -ErrorAction SilentlyContinue
    $consoleHwnd = [ClaudeNotify.ClaudeNotifyConsoleWindow]::GetConsoleWindow()
    if ($consoleHwnd -ne [IntPtr]::Zero) {
        [ClaudeNotify.ClaudeNotifyConsoleWindow]::ShowWindow($consoleHwnd, 0) | Out-Null
    }
} catch {}

if (-not $Relaunched) {
    # 最初の起動(hook本体)。標準入力の読み取りとログ記録だけ済ませたら、
    # 本処理(トースト表示・クリック待機)は完全に隠しウィンドウの子プロセスに
    # 任せてすぐ終了する。これでターミナルが起動しっぱなしに見えるのを防ぐ。
    $stdinReader = New-Object System.IO.StreamReader([Console]::OpenStandardInput(), [System.Text.Encoding]::UTF8)
    $stdin = $stdinReader.ReadToEnd()
    $title = 'Claude Code'
    $message = 'Claude Codeが承認を待っています'
    $cwd = ''
    $sessionId = ''

    try {
        $json = $stdin | ConvertFrom-Json
        if ($json.title) { $title = $json.title }
        if ($json.message) { $message = $json.message }
        if ($json.cwd) { $cwd = $json.cwd }
        if ($json.session_id) { $sessionId = $json.session_id }
    } catch {}

    # 子プロセスへはコマンドライン経由で渡すため、改行が混ざると引数が壊れる。
    # (Stopフックの要約など複数行のメッセージが来ることがある)
    $title = ($title -replace '[\r\n]+', ' ').Trim()
    $message = ($message -replace '[\r\n]+', ' ').Trim()

    # 切り分け用ログ(main/subagentの判別材料を後で確認するため生JSONを残す)
    try {
        $logEntry = (Get-Date -Format 'yyyy-MM-dd HH:mm:ss') + ' ' + $stdin
        Add-Content -Path "$HOME\.claude\notification-log.jsonl" -Value $logEntry -Encoding utf8
    } catch {}

    Write-DiagLog "parsed title='$title' message='$message' cwd='$cwd' sessionId='$sessionId'"

    # 子プロセス(トースト表示担当)はWindows PowerShell(powershell.exe)で起動する。
    # PowerShell 7(pwsh)はWinRTのトースト通知API読み込みに失敗することがあるため、
    # WinRT連携が安定しているレガシーのWindows PowerShell 5.1を使う。
    $exePath = "$env:WINDIR\System32\WindowsPowerShell\v1.0\powershell.exe"
    if (-not (Test-Path $exePath)) { $exePath = (Get-Process -Id $PID).Path }
    Write-DiagLog "exePath=$exePath PSCommandPath=$PSCommandPath"
    $argList = @('-NoProfile', '-NonInteractive', '-WindowStyle', 'Hidden', '-File', $PSCommandPath, '-Relaunched', '-Title', $title, '-Message', $message)
    if ($cwd) { $argList += @('-Cwd', $cwd) }
    if ($sessionId) { $argList += @('-SessionId', $sessionId) }

    # Start-Process -WindowStyle HiddenはWindows Terminalがデフォルト端末の環境だと
    # タスクバーに残ることがあるため、ProcessStartInfoでCreateNoWindowを明示して
    # 確実にウィンドウ/タスクバー表示なしで子プロセスを起動する。
    try {
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = $exePath
        $psi.Arguments = (($argList | ForEach-Object { ConvertTo-Win32Arg $_ }) -join ' ')
        $psi.UseShellExecute = $false
        $psi.CreateNoWindow = $true
        $psi.WindowStyle = [System.Diagnostics.ProcessWindowStyle]::Hidden
        $childProc = [System.Diagnostics.Process]::Start($psi)
        Write-DiagLog "child process started, PID=$($childProc.Id)"
    } catch {
        Write-DiagLog ("child process start FAILED: " + $_.Exception.ToString())
    }
    exit
}

# ここから先は隠しウィンドウの子プロセスでの本処理
$title = $Title
$message = $Message
$cwd = $Cwd
$sessionId = $SessionId

# 通知本文に手がかり(session_idの末尾・cwd)を添えて、パッと見で判別しやすくする
$detail = ''
if ($sessionId) {
    $detail += 'session: ...' + $sessionId.Substring([Math]::Max(0, $sessionId.Length - 8))
}
if ($cwd) {
    if ($detail) { $detail += ' / ' }
    $detail += 'cwd: ' + (Split-Path -Leaf $cwd)
}

Add-Type -AssemblyName System.Drawing

Add-Type @"
using System;
using System.Runtime.InteropServices;
public class ClaudeNotifyWin32 {
    [DllImport("user32.dll", SetLastError = true)] public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint processId);
    [DllImport("user32.dll", SetLastError = true)] public static extern bool AttachThreadInput(uint idAttach, uint idAttachTo, bool fAttach);
    [DllImport("user32.dll")] public static extern void SwitchToThisWindow(IntPtr hWnd, bool fAltTab);
    [DllImport("kernel32.dll")] public static extern uint GetCurrentThreadId();
    [DllImport("user32.dll")] public static extern void keybd_event(byte bVk, byte bScan, uint dwFlags, UIntPtr dwExtraInfo);
    [DllImport("user32.dll")] public static extern bool FlashWindowEx(ref FLASHWINFO pwfi);

    // SetForegroundWindowはOSの「フォーカス乗っ取り防止」により、直前に自プロセスが
    // 入力を処理していないと失敗する。Altキーの押下/離上を偽装して「直前に入力を処理した」
    // 状態を作ってから呼び出すことで許可されるようにする定番の回避策。
    // ただしトースト自体のクリックはShellExperienceHost経由で処理されるため、この方法は
    // OSの前面ロック解除タイミング次第で成功したりしなかったりする(ベストエフォート)。
    public static bool ForceForeground(IntPtr hWnd) {
        const byte VK_MENU = 0x12;
        const uint KEYEVENTF_KEYUP = 0x2;
        keybd_event(VK_MENU, 0, 0, UIntPtr.Zero);
        keybd_event(VK_MENU, 0, KEYEVENTF_KEYUP, UIntPtr.Zero);
        return SetForegroundWindow(hWnd);
    }

    // タスクバーアイコンの点滅はSetForegroundWindowと違いOSの前面ロックに関係なく必ず成功する。
    // ユーザーがクリックするまで点滅し続けるFLASHW_TIMERNOFGを使う。
    public static void FlashTaskbar(IntPtr hWnd) {
        FLASHWINFO fi = new FLASHWINFO();
        fi.cbSize = (uint)Marshal.SizeOf(typeof(FLASHWINFO));
        fi.hwnd = hWnd;
        fi.dwFlags = 0x00000003 /* FLASHW_ALL */ | 0x0000000C /* FLASHW_TIMERNOFG */;
        fi.uCount = 0;
        fi.dwTimeout = 0;
        FlashWindowEx(ref fi);
    }
}

[StructLayout(LayoutKind.Sequential)]
public struct FLASHWINFO {
    public uint cbSize;
    public IntPtr hwnd;
    public uint dwFlags;
    public uint uCount;
    public uint dwTimeout;
}
"@

function Get-ParentProcessId([int]$processId) {
    (Get-CimInstance Win32_Process -Filter "ProcessId=$processId" -ErrorAction SilentlyContinue).ParentProcessId
}

function Find-TargetWindowHandle {
    # 自プロセスから親を辿ってVSCode(Code.exe)のウィンドウを探す
    $currentId = $PID
    for ($i = 0; $i -lt 20; $i++) {
        $proc = Get-Process -Id $currentId -ErrorAction SilentlyContinue
        if ($proc -and $proc.ProcessName -eq 'Code' -and $proc.MainWindowHandle -ne [IntPtr]::Zero) {
            return $proc.MainWindowHandle
        }
        $parentId = Get-ParentProcessId $currentId
        if (-not $parentId -or $parentId -eq 0 -or $parentId -eq $currentId) { break }
        $currentId = $parentId
    }
    # 親を辿って見つからなければ、動いているVSCodeウィンドウから拾う(フォールバック)
    $codeProc = Get-Process -Name 'Code' -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne [IntPtr]::Zero } | Select-Object -First 1
    if ($codeProc) { return $codeProc.MainWindowHandle }
    return [IntPtr]::Zero
}

function Focus-TargetWindow {
    $hwnd = Find-TargetWindowHandle
    if ($hwnd -eq [IntPtr]::Zero) { return }

    if ([ClaudeNotifyWin32]::IsIconic($hwnd)) { [ClaudeNotifyWin32]::ShowWindow($hwnd, 9) }

    # タスクバー点滅は前面ロックに関係なく確実に効くので必ず実行する(ベースライン)。
    # SetForegroundWindow系は前面ロックの状態次第で成功しないことがあるためベストエフォート。
    [ClaudeNotifyWin32]::FlashTaskbar($hwnd)
    Write-DiagLog '  FlashTaskbar called'

    [ClaudeNotifyWin32]::SwitchToThisWindow($hwnd, $true)
    [ClaudeNotifyWin32]::ForceForeground($hwnd) | Out-Null

    Start-Sleep -Milliseconds 100
    $nowFg = [ClaudeNotifyWin32]::GetForegroundWindow()
    if ($nowFg -ne $hwnd) {
        # それでもダメならVSCode自身のCLI(-r: reuse window)にも依頼しておく
        $codeCmd = Get-Command 'code.cmd' -ErrorAction SilentlyContinue
        if (-not $codeCmd) { $codeCmd = Get-Command 'code' -ErrorAction SilentlyContinue }
        if ($codeCmd) {
            try {
                if ($cwd) {
                    Start-Process -FilePath $codeCmd.Source -ArgumentList @('-r', "`"$cwd`"") -WindowStyle Hidden
                } else {
                    Start-Process -FilePath $codeCmd.Source -ArgumentList @('-r') -WindowStyle Hidden
                }
            } catch {}
        }
    }
}

Write-DiagLog "child running, title='$title' message='$message' detail='$detail'"

$toastOk = $true
try {
    [void][Windows.UI.Notifications.ToastNotificationManager, Windows.UI.Notifications, ContentType = WindowsRuntime]
    [void][Windows.Data.Xml.Dom.XmlDocument, Windows.Data.Xml.Dom, ContentType = WindowsRuntime]

    # Windows標準の「Windows PowerShell」AppUserModelIDを間借りする定番手法(専用アプリ登録なしでトースト表示するため)
    $aumid = '{1AC14E77-02E7-4E5D-B744-2EB1AE5198B7}\WindowsPowerShell\v1.0\powershell.exe'

    $escTitle = [System.Security.SecurityElement]::Escape($title)
    $escBody1 = [System.Security.SecurityElement]::Escape($message)
    $escBody2 = [System.Security.SecurityElement]::Escape($detail)

    $xmlText = @"
<toast activationType="foreground" launch="claude-code-notify">
  <visual>
    <binding template="ToastGeneric">
      <text>$escTitle</text>
      <text>$escBody1</text>
      <text>$escBody2</text>
    </binding>
  </visual>
</toast>
"@

    $xmlDoc = New-Object Windows.Data.Xml.Dom.XmlDocument
    $xmlDoc.LoadXml($xmlText)
    $toast = New-Object Windows.UI.Notifications.ToastNotification $xmlDoc

    $script:claudeToastActivated = $false
    $toast.add_Activated({ $script:claudeToastActivated = $true })

    $notifier = [Windows.UI.Notifications.ToastNotificationManager]::CreateToastNotifier($aumid)
    $notifier.Show($toast)
    Write-DiagLog 'toast Show() called successfully'

    # Windows PowerShell(5.1)は既定でSTAスレッドのため、単純なStart-Sleepだけでは
    # メッセージポンプが回らず、クリック時のWinRTコールバック(Activated)が
    # ディスパッチされないことがある。DoEventsでメッセージポンプを回しながら待つ。
    Add-Type -AssemblyName System.Windows.Forms
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt 15 -and -not $script:claudeToastActivated) {
        [System.Windows.Forms.Application]::DoEvents()
        Start-Sleep -Milliseconds 100
    }

    if ($script:claudeToastActivated) {
        Write-DiagLog 'toast activated, focusing target window'
        Focus-TargetWindow
    } else {
        Write-DiagLog 'toast wait timed out without activation'
    }
} catch {
    $toastOk = $false
    Write-DiagLog ("WinRT toast FAILED: " + $_.Exception.ToString())
}

if (-not $toastOk) {
    Write-DiagLog 'falling back to legacy NotifyIcon balloon tip'
    # WinRTトーストが使えない環境向けのフォールバック(従来のバルーン通知)
    try {
        Add-Type -AssemblyName System.Windows.Forms
        $notifyIcon = New-Object System.Windows.Forms.NotifyIcon
        $notifyIcon.Icon = [System.Drawing.SystemIcons]::Information
        $notifyIcon.BalloonTipTitle = $title
        $notifyIcon.BalloonTipText = ($message + [Environment]::NewLine + $detail)
        $notifyIcon.Visible = $true
        $focusAction = { Focus-TargetWindow }
        $notifyIcon.add_BalloonTipClicked($focusAction)
        $notifyIcon.add_Click($focusAction)
        $notifyIcon.ShowBalloonTip(8000)
        Write-DiagLog 'NotifyIcon ShowBalloonTip() called successfully'
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        while ($sw.Elapsed.TotalSeconds -lt 10) {
            [System.Windows.Forms.Application]::DoEvents()
            Start-Sleep -Milliseconds 100
        }
        $notifyIcon.Dispose()
    } catch {
        Write-DiagLog ("NotifyIcon fallback FAILED: " + $_.Exception.ToString())
    }
}
Write-DiagLog 'script ending'
