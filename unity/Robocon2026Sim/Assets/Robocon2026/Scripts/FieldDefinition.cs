using System;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace Robocon2026.Simulation
{
    [Serializable]
    public sealed class FieldDefinition
    {
        [JsonProperty("schema_version")] public int SchemaVersion;
        [JsonProperty("name")] public string Name = string.Empty;
        [JsonProperty("units")] public string Units = string.Empty;
        [JsonProperty("coordinate_system")] public CoordinateSystemDefinition CoordinateSystem = new();
        [JsonProperty("field")] public FieldSizeDefinition Field = new();
        [JsonProperty("rules")] public RulesDefinition Rules = new();
        [JsonProperty("robot")] public RobotDefinition Robot = new();
        [JsonProperty("missions")] public MissionsDefinition Missions = new();
        [JsonProperty("zones")] public List<ZoneDefinition> Zones = new();
        [JsonProperty("walls")] public WallsDefinition Walls = new();
        [JsonProperty("bingo")] public BingoDefinition Bingo = new();
        [JsonProperty("notes")] public NotesDefinition Notes = new();

        public void Validate()
        {
            if (SchemaVersion != 1) throw new InvalidOperationException($"Unsupported field schema {SchemaVersion}.");
            if (Units != "metres") throw new InvalidOperationException($"Expected metres, got '{Units}'.");
            if (Field.Length <= 0 || Field.Width <= 0) throw new InvalidOperationException("Field dimensions must be positive.");
            if (Walls.Thickness <= 0 || Walls.Height <= 0) throw new InvalidOperationException("Wall dimensions must be positive.");
            if (Bingo.Columns != 3 || Bingo.Rows != 3) throw new InvalidOperationException("The competition bingo must be 3 x 3.");
            if (Robot.Chassis.Side <= 0 || Robot.Chassis.Height <= 0)
                throw new InvalidOperationException("Robot chassis dimensions must be positive.");
            if (Missions.StartToBingoLeft.Pickup.Count != 2 || Missions.StartToBingoLeft.Finish.Count != 2)
                throw new InvalidOperationException("Mission pickup and finish poses require x/y pairs.");

            RequireNear(3 * Bingo.ClearCell + 4 * Bingo.VerticalFrame, Bingo.Width, "bingo width");
            RequireNear(Bingo.BottomClearance + 3 * Bingo.ClearCell + 4 * Bingo.ShelfThickness,
                Bingo.Height, "bingo height");
            RequireNear(Bingo.Centre.X + Bingo.Width / 2, Bingo.FarEdgeX, "bingo far edge");
        }

        private static void RequireNear(double actual, double expected, string label)
        {
            if (Math.Abs(actual - expected) > 1e-6)
                throw new InvalidOperationException($"Invalid {label}: {actual:F6} != {expected:F6} m.");
        }
    }

    [Serializable] public sealed class CoordinateSystemDefinition
    {
        [JsonProperty("handedness")] public string Handedness = string.Empty;
        [JsonProperty("origin")] public string Origin = string.Empty;
    }

    [Serializable] public sealed class FieldSizeDefinition
    {
        [JsonProperty("length")] public double Length;
        [JsonProperty("width")] public double Width;
        [JsonProperty("floor_z")] public double FloorZ;
    }

    [Serializable] public sealed class RulesDefinition
    {
        [JsonProperty("note_diameter")] public double NoteDiameter;
        [JsonProperty("note_mass")] public double NoteMass;
        [JsonProperty("notes_per_team")] public int NotesPerTeam;
    }

    [Serializable] public sealed class ZoneDefinition
    {
        [JsonProperty("id")] public string Id = string.Empty;
        [JsonProperty("x_min")] public double XMin;
        [JsonProperty("x_max")] public double XMax;
        [JsonProperty("y_min")] public double YMin;
        [JsonProperty("y_max")] public double YMax;
        [JsonProperty("colour")] public string Colour = "#808080";
    }

    [Serializable] public sealed class RobotDefinition
    {
        [JsonProperty("chassis")] public ChassisDefinition Chassis = new();
        [JsonProperty("omni_wheels")] public OmniWheelsDefinition OmniWheels = new();
        [JsonProperty("target_limits")] public TargetLimitsDefinition TargetLimits = new();
    }

    [Serializable] public sealed class ChassisDefinition
    {
        [JsonProperty("shape")] public string Shape = string.Empty;
        [JsonProperty("side")] public double Side;
        [JsonProperty("height")] public double Height;
        [JsonProperty("corner_radius")] public double CornerRadius;
    }

    [Serializable] public sealed class OmniWheelsDefinition
    {
        [JsonProperty("count")] public int Count;
        [JsonProperty("diameter_options")] public List<double> DiameterOptions = new();
        [JsonProperty("simulation_diameter")] public double SimulationDiameter;
        [JsonProperty("width")] public double Width;
    }

    [Serializable] public sealed class TargetLimitsDefinition
    {
        [JsonProperty("max_speed")] public double MaxSpeed;
        [JsonProperty("max_acceleration")] public double MaxAcceleration;
        [JsonProperty("max_yaw_rate")] public double MaxYawRate;
    }

    [Serializable] public sealed class MissionsDefinition
    {
        [JsonProperty("start_to_bingo_left")] public MissionDefinition StartToBingoLeft = new();
    }

    [Serializable] public sealed class MissionDefinition
    {
        [JsonProperty("surface_clearance")] public double SurfaceClearance;
        [JsonProperty("yaw")] public double Yaw;
        [JsonProperty("start")] public List<double> Start = new();
        [JsonProperty("pickup")] public List<double> Pickup = new();
        [JsonProperty("slalom_waypoints")] public List<List<double>> SlalomWaypoints = new();
        [JsonProperty("finish")] public List<double> Finish = new();
        [JsonProperty("pickup_note_indices")] public List<int> PickupNoteIndices = new();
        [JsonProperty("pickup_dwell_s")] public double PickupDwellSeconds;
    }

    [Serializable] public sealed class WallsDefinition
    {
        [JsonProperty("thickness")] public double Thickness;
        [JsonProperty("height")] public double Height;
        [JsonProperty("segments")] public List<WallSegmentDefinition> Segments = new();
    }

    [Serializable] public sealed class WallSegmentDefinition
    {
        [JsonProperty("id")] public string Id = string.Empty;
        [JsonProperty("x1")] public double X1;
        [JsonProperty("y1")] public double Y1;
        [JsonProperty("x2")] public double X2;
        [JsonProperty("y2")] public double Y2;
    }

    [Serializable] public sealed class Point2Definition
    {
        [JsonProperty("x")] public double X;
        [JsonProperty("y")] public double Y;
    }

    [Serializable] public sealed class BingoDefinition
    {
        [JsonProperty("centre")] public Point2Definition Centre = new();
        [JsonProperty("columns")] public int Columns;
        [JsonProperty("rows")] public int Rows;
        [JsonProperty("clear_cell")] public double ClearCell;
        [JsonProperty("vertical_frame")] public double VerticalFrame;
        [JsonProperty("shelf_thickness")] public double ShelfThickness;
        [JsonProperty("bottom_clearance")] public double BottomClearance;
        [JsonProperty("width")] public double Width;
        [JsonProperty("depth_each_side")] public double DepthEachSide;
        [JsonProperty("total_depth")] public double TotalDepth;
        [JsonProperty("height")] public double Height;
        [JsonProperty("far_edge_x")] public double FarEdgeX;
    }

    [Serializable] public sealed class NotesDefinition
    {
        [JsonProperty("blue")] public List<List<double>> Blue = new();
        [JsonProperty("orange")] public List<List<double>> Orange = new();
    }
}
