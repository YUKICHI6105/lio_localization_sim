using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace Robocon2026.Simulation.Editor
{
    public static class ProjectSetup
    {
        private const string SettingsDirectory = "Assets/Robocon2026/Settings";
        private const string PipelinePath = SettingsDirectory + "/Robocon2026URP.asset";
        private const string SceneDirectory = "Assets/Robocon2026/Scenes";
        private const string ScenePath = SceneDirectory + "/Robocon2026Simulation.unity";

        [MenuItem("Robocon 2026/Configure URP and Simulation Scene")]
        public static void Configure()
        {
            Directory.CreateDirectory(SettingsDirectory);
            Directory.CreateDirectory(SceneDirectory);

            var pipeline = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>(PipelinePath);
            if (pipeline == null)
            {
                pipeline = ScriptableObject.CreateInstance<UniversalRenderPipelineAsset>();
                pipeline.name = "Robocon2026URP";
                pipeline.LoadBuiltinRendererData(RendererType.UniversalRenderer);
                pipeline.renderScale = 1f;
                pipeline.msaaSampleCount = 4;
                pipeline.supportsHDR = false;
                pipeline.useSRPBatcher = true;
                AssetDatabase.CreateAsset(pipeline, PipelinePath);
            }

            GraphicsSettings.defaultRenderPipeline = pipeline;
            QualitySettings.renderPipeline = pipeline;
            pipeline.msaaSampleCount = 4;

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = new GameObject("Robocon2026Simulation");
            root.AddComponent<SimulationBootstrap>();
            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };

            AssetDatabase.SaveAssets();
            Selection.activeObject = pipeline;
            Debug.Log("[Robocon2026] URP and simulation scene configured. Press Play to build the field.");
        }

        [MenuItem("Robocon 2026/Validate Generated Field")]
        public static void ValidateGeneratedField()
        {
            var sceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(ScenePath);
            if (sceneAsset == null)
                throw new FileNotFoundException("Configure the simulation scene first.", ScenePath);

            EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            var bootstrap = Object.FindAnyObjectByType<SimulationBootstrap>();
            if (bootstrap == null)
                throw new MissingComponentException("SimulationBootstrap is missing.");

            var builder = bootstrap.GetComponent<RoboconFieldBuilder>()
                ?? bootstrap.gameObject.AddComponent<RoboconFieldBuilder>();
            builder.Build();
            Physics.SyncTransforms();

            var generated = bootstrap.transform.Find("GeneratedField")
                ?? throw new MissingReferenceException("GeneratedField was not created.");
            RequireChildren(generated, "BlueNotes", 6);
            RequireChildren(generated, "OrangeNotes", 6);
            RequireNear(generated.Find("Bingo/Post_0").position,
                FieldCoordinates.ToUnity(1.866, 0, 0.47), "bingo post 0");
            RequireNear(generated.Find("Bingo/Shelf_3").position,
                FieldCoordinates.ToUnity(2.316, 0, 0.89), "bingo top shelf");

            var colliders = generated.GetComponentsInChildren<Collider>(true);
            if (colliders.Length < 30)
                throw new MissingComponentException($"Expected at least 30 colliders, got {colliders.Length}.");

            Debug.Log($"[Robocon2026 validation] field generated: {colliders.Length} colliders, " +
                "12 notes, bingo placement OK.");
        }

        private static void RequireChildren(Transform root, string path, int expected)
        {
            var item = root.Find(path) ?? throw new MissingReferenceException(path);
            if (item.childCount != expected)
                throw new System.InvalidOperationException(
                    $"{path}: expected {expected} children, got {item.childCount}.");
        }

        private static void RequireNear(Vector3 actual, Vector3 expected, string label)
        {
            if ((actual - expected).sqrMagnitude > 1e-10f)
                throw new System.InvalidOperationException(
                    $"{label}: expected {expected}, got {actual}.");
        }
    }
}
