using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

namespace UnityRobotEnv.Core
{
    public static class RegionSampler
    {
        public static float SampleX(BDDL.BDDLRegion region)
        {
            if (region.Ranges.Count == 0) return 0f;
            var range = region.Ranges[Random.Range(0, region.Ranges.Count)];
            return Random.Range(range.XMin, range.XMax);
        }

        public static float SampleY(BDDL.BDDLRegion region)
        {
            if (region.Ranges.Count == 0) return 0f;
            var range = region.Ranges[Random.Range(0, region.Ranges.Count)];
            return Random.Range(range.YMin, range.YMax);
        }

        public static float SampleYaw(BDDL.BDDLRegion region)
        {
            if (region.YawRotation.Count >= 2)
                return Random.Range(region.YawRotation[0], region.YawRotation[1]);
            return 0f;
        }
    }
}
