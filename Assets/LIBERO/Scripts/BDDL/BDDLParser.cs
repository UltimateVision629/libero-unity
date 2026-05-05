using System;
using System.Collections.Generic;
using System.IO;

namespace LIBERO.BDDL
{
    public class BDDLParser
    {
        public static BDDLProblem ParseFile(string filePath)
        {
            if (!File.Exists(filePath))
                throw new FileNotFoundException($"BDDL file not found: {filePath}");
            return Parse(File.ReadAllText(filePath));
        }

        public static BDDLProblem Parse(string bddlText)
        {
            var tokenizer = new BDDLTokenizer(bddlText);
            var tokens = tokenizer.TokenizeAll();
            var sExprParser = new BDDLSExprParser(tokens);
            var exprs = sExprParser.ParseAll();

            if (exprs.Count == 0)
                throw new BDDLParseException("Empty BDDL file", 1, 1);

            var defineExpr = exprs[0] as SList;
            if (defineExpr == null || defineExpr.Count < 3)
                throw new BDDLParseException("Malformed (define ...) expression", 1, 1);

            var defineSym = (defineExpr[0] as SAtom)?.Value;
            if (defineSym != "define")
                throw new BDDLParseException("Expected 'define' at top level", 1, 1);

            var problemSpec = defineExpr[1] as SList;
            if (problemSpec == null || problemSpec.Count < 2)
                throw new BDDLParseException("Malformed (problem ...) expression", 1, 1);

            var problemSym = (problemSpec[0] as SAtom)?.Value;
            if (problemSym != "problem")
                throw new BDDLParseException("Expected 'problem' in define", 1, 1);

            var problem = new BDDLProblem
            {
                ProblemName = (problemSpec[1] as SAtom)?.Value ?? "unknown"
            };

            var body = defineExpr.Children.GetRange(2, defineExpr.Count - 2);
            ParseBody(problem, body);

            return problem;
        }

        private static void ParseBody(BDDLProblem problem, List<SExpr> body)
        {
            for (int i = 0; i < body.Count; i++)
            {
                string key = null;

                if (body[i] is SAtom atom && atom.IsSymbol && atom.Value.StartsWith(":"))
                    key = atom.Value;
                else if (body[i] is SList list && list.Count > 0 && list[0] is SAtom listHead && listHead.IsSymbol && listHead.Value.StartsWith(":"))
                {
                    key = listHead.Value;
                    ParseSection(problem, key, list.Children.GetRange(1, list.Count - 1), ref i);
                    continue;
                }

                if (key == null) continue;

                switch (key)
                {
                    case ":domain":
                        if (i + 1 < body.Count && body[i + 1] is SAtom domain)
                            problem.DomainName = domain.Value;
                        i++;
                        break;
                    case ":language":
                        {
                            var words = new System.Collections.Generic.List<string>();
                            int j = i + 1;
                            while (j < body.Count && body[j] is SAtom word && !word.Value.StartsWith(":"))
                            {
                                words.Add(word.Value);
                                j++;
                            }
                            problem.LanguageInstruction = string.Join(" ", words);
                            i = j - 1;
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        private static void ParseSection(BDDLProblem problem, string key, List<SExpr> children, ref int blockIndex)
        {
            switch (key)
            {
                case ":domain":
                    if (children.Count > 0 && children[0] is SAtom domainAtom)
                        problem.DomainName = domainAtom.Value;
                    break;
                case ":language":
                    {
                        var words = new System.Collections.Generic.List<string>();
                        foreach (var child in children)
                        {
                            if (child is SAtom word && !word.Value.StartsWith(":"))
                                words.Add(word.Value);
                            else break;
                        }
                        problem.LanguageInstruction = string.Join(" ", words);
                    }
                    break;
                case ":regions":
                    ParseRegions(problem, children, 0);
                    break;
                case ":fixtures":
                    ParseFixtures(problem, children, 0);
                    break;
                case ":objects":
                    ParseObjects(problem, children, 0);
                    break;
                case ":obj_of_interest":
                    ParseObjOfInterest(problem, children, 0);
                    break;
                case ":init":
                    ParseInit(problem, children, 0);
                    break;
                case ":goal":
                    ParseGoal(problem, children, 0);
                    break;
            }
        }

        private static int ParseRegions(BDDLProblem problem, List<SExpr> body, int startIdx)
        {
            int i = startIdx;
            while (i < body.Count && body[i] is SList regionDef)
            {
                if (regionDef.Count < 3) break;
                var regionName = (regionDef[0] as SAtom)?.Value;
                if (string.IsNullOrWhiteSpace(regionName)) break;

                var region = new BDDLRegion(regionName);

                for (int j = 1; j < regionDef.Count; j++)
                {
                    if (!(regionDef[j] is SList propList) || propList.Count < 2) continue;
                    var propKey = (propList[0] as SAtom)?.Value;

                    switch (propKey)
                    {
                        case ":target":
                            region.Target = (propList[1] as SAtom)?.Value ?? "";
                            break;
                        case ":ranges":
                            for (int k = 1; k < propList.Count; k++)
                            {
                                if (!(propList[k] is SList outerRange)) continue;
                                for (int inner = 0; inner < outerRange.Count; inner++)
                                {
                                    if (outerRange[inner] is SList rangeList && rangeList.Count == 4)
                                    {
                                        float[] vals = new float[4];
                                        bool valid = true;
                                        for (int m = 0; m < 4; m++)
                                        {
                                            if (rangeList[m] is SAtom atom)
                                                valid &= float.TryParse(atom.Value,
                                                    System.Globalization.NumberStyles.Float,
                                                    System.Globalization.CultureInfo.InvariantCulture,
                                                    out vals[m]);
                                            else valid = false;
                                        }
                                        if (valid)
                                            region.Ranges.Add(new RegionRange(vals[0], vals[1], vals[2], vals[3]));
                                    }
                                }
                            }
                            break;
                        case ":yaw_rotation":
                            for (int k = 1; k < propList.Count; k++)
                            {
                                if (!(propList[k] is SList outerYaw)) continue;
                                for (int inner = 0; inner < outerYaw.Count; inner++)
                                {
                                    if (outerYaw[inner] is SList yawList && yawList.Count == 2)
                                    {
                                        if (float.TryParse((yawList[0] as SAtom)?.Value,
                                                System.Globalization.NumberStyles.Float,
                                                System.Globalization.CultureInfo.InvariantCulture,
                                                out float yawMin) &&
                                            float.TryParse((yawList[1] as SAtom)?.Value,
                                                System.Globalization.NumberStyles.Float,
                                                System.Globalization.CultureInfo.InvariantCulture,
                                                out float yawMax))
                                        {
                                            region.YawRotation.Add(yawMin);
                                            region.YawRotation.Add(yawMax);
                                        }
                                    }
                                }
                            }
                            break;
                    }
                }
                string dictKey = string.IsNullOrEmpty(region.Target) ? regionName : region.Target + "_" + regionName;
                problem.Regions[dictKey] = region;
                i++;
            }
            return i - 1;
        }

        private static int ParseFixtures(BDDLProblem problem, List<SExpr> body, int startIdx)
        {
            int i = startIdx;
            while (i + 2 < body.Count &&
                   body[i] is SAtom fName && fName.IsSymbol &&
                   body[i + 1] is SAtom sep && sep.Value == "-" &&
                   body[i + 2] is SAtom fCat && fCat.IsSymbol)
            {
                problem.Fixtures[fName.Value] = fCat.Value;
                i += 3;
            }
            return i;
        }

        private static int ParseObjects(BDDLProblem problem, List<SExpr> body, int startIdx)
        {
            int i = startIdx;
            while (i < body.Count)
            {
                var names = new List<string>();
                while (i < body.Count && body[i] is SAtom token && token.IsSymbol && token.Value != "-")
                {
                    names.Add(token.Value);
                    i++;
                }
                if (i < body.Count && body[i] is SAtom dash && dash.Value == "-")
                    i++;
                if (i < body.Count && body[i] is SAtom catToken && catToken.IsSymbol)
                {
                    foreach (var name in names)
                        problem.Objects[name] = catToken.Value;
                    i++;
                }
                else
                {
                    break;
                }
            }
            return i;
        }

        private static int ParseObjOfInterest(BDDLProblem problem, List<SExpr> body, int startIdx)
        {
            int i = startIdx;
            while (i < body.Count && body[i] is SAtom token && token.IsSymbol)
            {
                problem.ObjOfInterest.Add(token.Value);
                i++;
            }
            return i - 1;
        }

        private static int ParseInit(BDDLProblem problem, List<SExpr> body, int startIdx)
        {
            int i = startIdx;
            while (i < body.Count && body[i] is SList initExpr && initExpr.Count >= 2)
            {
                var predName = (initExpr[0] as SAtom)?.Value;
                if (string.IsNullOrWhiteSpace(predName)) break;

                var state = new InitState();

                switch (predName.ToLowerInvariant())
                {
                    case "on":
                        state.Predicate = InitPredicateType.On;
                        state.ObjectName = (initExpr[1] as SAtom)?.Value ?? "";
                        state.LocationName = initExpr.Count >= 3 ? ((initExpr[2] as SAtom)?.Value ?? "") : "";
                        break;
                    case "in":
                        state.Predicate = InitPredicateType.In;
                        state.ObjectName = (initExpr[1] as SAtom)?.Value ?? "";
                        state.LocationName = initExpr.Count >= 3 ? ((initExpr[2] as SAtom)?.Value ?? "") : "";
                        break;
                    case "open":
                        state.Predicate = InitPredicateType.Open;
                        state.ObjectName = (initExpr[1] as SAtom)?.Value ?? "";
                        break;
                    case "close":
                        state.Predicate = InitPredicateType.Close;
                        state.ObjectName = (initExpr[1] as SAtom)?.Value ?? "";
                        break;
                    default:
                        break;
                }
                problem.InitialState.Add(state);
                i++;
            }
            return i - 1;
        }

        private static int ParseGoal(BDDLProblem problem, List<SExpr> body, int startIdx)
        {
            if (startIdx < body.Count && body[startIdx] is SList goalExpr)
            {
                var goalType = (goalExpr[0] as SAtom)?.Value;
                if (goalType == "And" || goalType == "and")
                {
                    problem.Goal = new AndGoal { Conditions = new List<GoalCondition>() };
                    for (int j = 1; j < goalExpr.Count; j++)
                    {
                        if (goalExpr[j] is SList condExpr && condExpr.Count >= 3)
                        {
                            var condType = (condExpr[0] as SAtom)?.Value?.ToLowerInvariant();
                            if (condType == "on")
                            {
                                problem.Goal.Conditions.Add(new GoalCondition
                                {
                                    Type = GoalType.On,
                                    ObjectName = (condExpr[1] as SAtom)?.Value ?? "",
                                    TargetName = (condExpr[2] as SAtom)?.Value ?? ""
                                });
                            }
                        }
                    }
                }
            }
            return startIdx;
        }
    }
}
