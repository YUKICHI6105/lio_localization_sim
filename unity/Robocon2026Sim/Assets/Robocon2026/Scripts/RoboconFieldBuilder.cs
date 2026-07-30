using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace Robocon2026.Simulation
{
    public sealed class RoboconFieldBuilder : MonoBehaviour
    {
        public const string DefinitionFileName = "robocon2026_field.json";
        public const string DisableNotesExperimentFlag =
            "DisableNotesForLocalizationExperiment.flag";

        private readonly Dictionary<string, Material> materials = new();
        private PhysicsMaterial fieldPhysicsMaterial = null!;
        private Transform generatedRoot = null!;
        private bool disableNotesForLocalizationExperiment;

        public FieldDefinition Definition { get; private set; } = null!;

        public void Build()
        {
            Definition = LoadDefinition();
            Definition.Validate();
            ConfigurePhysics();
            disableNotesForLocalizationExperiment = File.Exists(Path.GetFullPath(
                Path.Combine(Application.dataPath, "..", DisableNotesExperimentFlag)));

            var oldRoot = transform.Find("GeneratedField");
            if (oldRoot != null) Destroy(oldRoot.gameObject);
            generatedRoot = new GameObject("GeneratedField").transform;
            generatedRoot.SetParent(transform, false);

            fieldPhysicsMaterial = new PhysicsMaterial("FieldInitialCalibration")
            {
                staticFriction = 0.65f,
                dynamicFriction = 0.55f,
                bounciness = 0.05f,
                frictionCombine = PhysicsMaterialCombine.Average,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };

            BuildFloor();
            BuildZones();
            BuildWalls();
            BuildBingo();
            BuildNotes("Blue", Definition.Notes.Blue, "#1260FF");
            BuildNotes("Orange", Definition.Notes.Orange, "#FF6412");
            BuildView();

            if (disableNotesForLocalizationExperiment)
                Debug.LogWarning(
                    $"[Robocon2026] Localization experiment: all note GameObjects are inactive " +
                    $"because {DisableNotesExperimentFlag} exists.");
            Debug.Log($"[Robocon2026] Built '{Definition.Name}' from {DefinitionFileName}.");
        }

        private static FieldDefinition LoadDefinition()
        {
            var path = Path.Combine(Application.streamingAssetsPath, DefinitionFileName);
            if (!File.Exists(path))
                throw new FileNotFoundException("Run tools/sync_unity_field.py before opening Unity.", path);
            var definition = JsonConvert.DeserializeObject<FieldDefinition>(File.ReadAllText(path));
            return definition ?? throw new InvalidDataException($"Could not deserialize {path}.");
        }

        private static void ConfigurePhysics()
        {
            Time.fixedDeltaTime = 0.001f; // 1 kHz, matching the specified physical IMU.
            Time.maximumDeltaTime = 0.020f;
            Physics.simulationMode = SimulationMode.FixedUpdate;
            Physics.gravity = new Vector3(0f, -9.80665f, 0f);
            Physics.defaultSolverIterations = 12;
            Physics.defaultSolverVelocityIterations = 4;
            Physics.defaultContactOffset = 0.002f;
            Physics.defaultMaxDepenetrationVelocity = 5f;
            Physics.bounceThreshold = 0.5f;
            Physics.reuseCollisionCallbacks = true;

            Application.targetFrameRate = 60;
            Application.runInBackground = true;
            QualitySettings.vSyncCount = 1;
        }

        private void BuildFloor()
        {
            var f = Definition.Field;
            CreateBox("Floor", 0, 0, -0.01, f.Length, f.Width, 0.02,
                "#D6D6D6", true, generatedRoot);
        }

        private void BuildZones()
        {
            var root = NewGroup("Zones");
            foreach (var zone in Definition.Zones)
            {
                var x = (zone.XMin + zone.XMax) / 2;
                var y = (zone.YMin + zone.YMax) / 2;
                CreateBox(zone.Id, x, y, 0.001,
                    zone.XMax - zone.XMin, zone.YMax - zone.YMin, 0.002,
                    zone.Colour, false, root);
            }
        }

        private void BuildWalls()
        {
            var root = NewGroup("Walls");
            foreach (var wall in Definition.Walls.Segments)
            {
                var a = FieldCoordinates.ToUnity(wall.X1, wall.Y1, Definition.Walls.Height / 2);
                var b = FieldCoordinates.ToUnity(wall.X2, wall.Y2, Definition.Walls.Height / 2);
                var direction = b - a;
                var go = CreateCube(wall.Id, "#F0F0F0", true, root);
                go.transform.position = (a + b) / 2;
                go.transform.rotation = Quaternion.LookRotation(direction.normalized, Vector3.up);
                go.transform.localScale = new Vector3(
                    (float)Definition.Walls.Thickness,
                    (float)Definition.Walls.Height,
                    direction.magnitude);
            }
        }

        private void BuildBingo()
        {
            var root = NewGroup("Bingo");
            var bingo = Definition.Bingo;
            var pitch = bingo.ClearCell + bingo.VerticalFrame;
            var postHeight = bingo.Height - bingo.BottomClearance;
            var postZ = bingo.BottomClearance + postHeight / 2;

            for (var i = 0; i <= bingo.Columns; ++i)
            {
                var localX = -bingo.Width / 2 + bingo.VerticalFrame / 2 + i * pitch;
                CreateBox($"Post_{i}", bingo.Centre.X + localX, bingo.Centre.Y, postZ,
                    bingo.VerticalFrame, bingo.TotalDepth, postHeight,
                    "#D0D5DD", true, root);
            }

            var verticalPitch = bingo.ClearCell + bingo.ShelfThickness;
            for (var i = 0; i <= bingo.Rows; ++i)
            {
                var z = bingo.BottomClearance + bingo.ShelfThickness / 2 + i * verticalPitch;
                CreateBox($"Shelf_{i}", bingo.Centre.X, bingo.Centre.Y, z,
                    bingo.Width, bingo.TotalDepth, bingo.ShelfThickness,
                    "#D0D5DD", true, root);
            }
        }

        private void BuildNotes(string team, IReadOnlyList<List<double>> notes, string colour)
        {
            var root = NewGroup($"{team}Notes");
            for (var i = 0; i < notes.Count; ++i)
            {
                if (notes[i].Count < 2) throw new InvalidDataException($"{team} note {i} has no x/y pair.");
                var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = $"{team}Note_{i}";
                go.transform.SetParent(root, false);
                go.transform.position = FieldCoordinates.ToUnity(
                    notes[i][0], notes[i][1], Definition.Rules.NoteDiameter / 2);
                go.transform.localScale = Vector3.one * (float)Definition.Rules.NoteDiameter;
                go.GetComponent<Renderer>().sharedMaterial = GetMaterial(colour);
                go.GetComponent<Collider>().sharedMaterial = fieldPhysicsMaterial;

                var body = go.AddComponent<Rigidbody>();
                body.mass = (float)Definition.Rules.NoteMass;
                body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                body.interpolation = RigidbodyInterpolation.Interpolate;
                body.linearDamping = 0.02f;
                body.angularDamping = 0.05f;
                body.solverIterations = 12;
                body.solverVelocityIterations = 4;

                // Keep the generated hierarchy intact for editor validation, while removing
                // rendering, raycast and collision participation for the controlled
                // localization A/B experiment. The default remains the competition scene.
                if (disableNotesForLocalizationExperiment)
                    go.SetActive(false);
            }
        }

        private GameObject CreateBox(string name, double x, double y, double z,
            double sizeX, double sizeY, double sizeZ, string colour, bool collision, Transform parent)
        {
            var go = CreateCube(name, colour, collision, parent);
            go.transform.position = FieldCoordinates.ToUnity(x, y, z);
            go.transform.localScale = FieldCoordinates.SizeToUnity(sizeX, sizeY, sizeZ);
            return go;
        }

        private GameObject CreateCube(string name, string colour, bool collision, Transform parent)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.isStatic = true;

            var renderer = go.GetComponent<MeshRenderer>();
            renderer.sharedMaterial = GetMaterial(colour);
            renderer.shadowCastingMode = ShadowCastingMode.Off;
            renderer.receiveShadows = false;

            var collider = go.GetComponent<BoxCollider>();
            if (collision) collider.sharedMaterial = fieldPhysicsMaterial;
            else collider.enabled = false;
            return go;
        }

        private Material GetMaterial(string htmlColour)
        {
            if (materials.TryGetValue(htmlColour, out var existing)) return existing;
            if (!ColorUtility.TryParseHtmlString(htmlColour, out var colour)) colour = Color.gray;

            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            var material = new Material(shader)
            {
                name = $"Field_{htmlColour.TrimStart('#')}",
                color = colour,
                enableInstancing = true
            };
            if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", colour);
            materials.Add(htmlColour, material);
            return material;
        }

        private Transform NewGroup(string name)
        {
            var group = new GameObject(name).transform;
            group.SetParent(generatedRoot, false);
            return group;
        }

        private static void BuildView()
        {
            var camera = FindAnyObjectByType<Camera>();
            if (camera == null)
            {
                var cameraObject = new GameObject("SimulationCamera");
                camera = cameraObject.AddComponent<Camera>();
                cameraObject.tag = "MainCamera";
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = new Color(0.65f, 0.78f, 0.90f);
                camera.nearClipPlane = 0.05f;
                camera.farClipPlane = 100f;
                camera.transform.position = new Vector3(4.8f, 5.5f, -5.5f);
                camera.transform.LookAt(new Vector3(0f, 0f, 0.2f));
            }

            if (camera.GetComponent<SimulationCameraController>() == null)
                camera.gameObject.AddComponent<SimulationCameraController>();

            var cameraData = camera.GetUniversalAdditionalCameraData();
            cameraData.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            cameraData.antialiasingQuality = AntialiasingQuality.High;
            cameraData.renderPostProcessing = true;

            if (FindAnyObjectByType<Light>() == null)
            {
                var lightObject = new GameObject("Sun");
                var light = lightObject.AddComponent<Light>();
                light.type = LightType.Directional;
                light.intensity = 1.2f;
                light.shadows = LightShadows.None;
                lightObject.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            }
        }
    }
}
