using System;
using System.Reflection;
using UnityEditor;
using UnityEngine;

namespace Robocon2026.Simulation.Editor
{
    public static class GameViewResolutionSetup
    {
        private const int TargetWidth = 1920;
        private const int TargetHeight = 1080;

        [MenuItem("Robocon 2026/Set Game View to Full HD 1080p")]
        public static void Set1080p()
        {
            var editorAssembly = typeof(EditorWindow).Assembly;
            var gameViewType = editorAssembly.GetType("UnityEditor.GameView", true);
            var gameViewSizesType = editorAssembly.GetType("UnityEditor.GameViewSizes", true);
            var gameViewSizeType = editorAssembly.GetType("UnityEditor.GameViewSize", true);
            var gameViewSizeEnumType = editorAssembly.GetType("UnityEditor.GameViewSizeType", true);

            var singletonType = typeof(ScriptableSingleton<>).MakeGenericType(gameViewSizesType);
            var sizes = singletonType.GetProperty("instance", BindingFlags.Public | BindingFlags.Static)!
                .GetValue(null);
            var groupType = gameViewSizesType
                .GetProperty("currentGroupType", BindingFlags.Public | BindingFlags.Instance)!
                .GetValue(sizes);
            var group = gameViewSizesType
                .GetMethod("GetGroup", BindingFlags.Public | BindingFlags.Instance)!
                .Invoke(sizes, new[] { groupType });

            var groupRuntimeType = group!.GetType();
            var getBuiltinCount = groupRuntimeType.GetMethod("GetBuiltinCount")!;
            var getCustomCount = groupRuntimeType.GetMethod("GetCustomCount")!;
            var getGameViewSize = groupRuntimeType.GetMethod("GetGameViewSize")!;
            var builtinCount = (int)getBuiltinCount.Invoke(group, null)!;
            var totalCount = builtinCount + (int)getCustomCount.Invoke(group, null)!;
            var selectedIndex = -1;

            for (var i = 0; i < totalCount; ++i)
            {
                var size = getGameViewSize.Invoke(group, new object[] { i })!;
                var width = (int)gameViewSizeType.GetProperty("width")!.GetValue(size)!;
                var height = (int)gameViewSizeType.GetProperty("height")!.GetValue(size)!;
                if (width == TargetWidth && height == TargetHeight)
                {
                    selectedIndex = i;
                    break;
                }
            }

            if (selectedIndex < 0)
            {
                var fixedResolution = Enum.Parse(gameViewSizeEnumType, "FixedResolution");
                var newSize = Activator.CreateInstance(gameViewSizeType,
                    fixedResolution, TargetWidth, TargetHeight, "Full HD 1080p")!;
                groupRuntimeType.GetMethod("AddCustomSize")!.Invoke(group, new[] { newSize });
                selectedIndex = totalCount;
            }

            var gameView = EditorWindow.GetWindow(gameViewType);
            gameViewType.GetProperty("selectedSizeIndex",
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance)!
                .SetValue(gameView, selectedIndex);
            gameView.Repaint();
            gameView.ShowNotification(new GUIContent("Rendering at 1920 x 1080"));
            Debug.Log("[Robocon2026] Game View fixed to 1920 x 1080 (Full HD).");
        }
    }
}
