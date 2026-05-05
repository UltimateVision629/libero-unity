using System.Collections.Generic;

namespace LIBERO.BDDL
{
    public struct RegionRange
    {
        public float XMin, YMin, XMax, YMax;

        public RegionRange(float xMin, float yMin, float xMax, float yMax)
        {
            XMin = xMin; YMin = yMin; XMax = xMax; YMax = yMax;
        }

        public float Width => XMax - XMin;
        public float Height => YMax - YMin;

        public override string ToString() => $"({XMin:F3} {YMin:F3} {XMax:F3} {YMax:F3})";
    }

    public class BDDLRegion
    {
        public string Name;
        public string Target;
        public List<RegionRange> Ranges;
        public List<float> YawRotation;

        public BDDLRegion(string name)
        {
            Name = name;
            Ranges = new List<RegionRange>();
            YawRotation = new List<float>();
        }
    }

    public enum InitPredicateType
    {
        On,    // (On object_name region_or_object_name)
        In,    // (In object_name region_name)
        Open,  // (Open fixture_name)
        Close, // (Close fixture_name)
        TurnOn,
        TurnOff
    }

    public struct InitState
    {
        public InitPredicateType Predicate;
        public string ObjectName;
        public string LocationName; // region name or other object name
    }

    public enum GoalType
    {
        On
    }

    public struct GoalCondition
    {
        public GoalType Type;
        public string ObjectName;
        public string TargetName;
    }

    public struct AndGoal
    {
        public List<GoalCondition> Conditions;
    }

    public class BDDLProblem
    {
        public string ProblemName;
        public string DomainName;
        public string LanguageInstruction;

        public Dictionary<string, BDDLRegion> Regions;
        public Dictionary<string, string> Fixtures;     // name → category
        public Dictionary<string, string> Objects;      // name → category
        public List<string> ObjOfInterest;

        public List<InitState> InitialState;
        public AndGoal Goal;

        public BDDLProblem()
        {
            Regions = new Dictionary<string, BDDLRegion>();
            Fixtures = new Dictionary<string, string>();
            Objects = new Dictionary<string, string>();
            ObjOfInterest = new List<string>();
            InitialState = new List<InitState>();
        }

        public override string ToString()
        {
            var lines = new List<string>
            {
                $"Problem: {ProblemName}",
                $"Domain: {DomainName}",
                $"Language: {LanguageInstruction}",
                $"Regions ({Regions.Count}):"
            };
            foreach (var r in Regions.Values)
                lines.Add($"  {r.Name} → {r.Target} [{r.Ranges.Count} ranges]");
            lines.Add($"Fixtures ({Fixtures.Count}):");
            foreach (var f in Fixtures)
                lines.Add($"  {f.Key} : {f.Value}");
            lines.Add($"Objects ({Objects.Count}):");
            foreach (var o in Objects)
                lines.Add($"  {o.Key} : {o.Value}");
            lines.Add($"ObjOfInterest ({ObjOfInterest.Count}):");
            foreach (var oi in ObjOfInterest)
                lines.Add($"  {oi}");
            lines.Add($"Init States ({InitialState.Count}):");
            foreach (var s in InitialState)
                lines.Add($"  ({s.Predicate} {s.ObjectName} {s.LocationName})");
            lines.Add($"Goal: (And");
            foreach (var g in Goal.Conditions)
                lines.Add($"  ({g.Type} {g.ObjectName} {g.TargetName})");
            lines.Add(")");
            return string.Join("\n", lines);
        }
    }
}
