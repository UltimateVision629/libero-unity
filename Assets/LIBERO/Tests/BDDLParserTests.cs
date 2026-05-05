using System.Collections;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

using LIBERO.BDDL;
using LIBERO.Core;

namespace LIBERO.Tests
{
    public class BDDLParserTests
    {
        private static readonly string SampleBDDL = @"
(define (problem LIBERO_Tabletop_Manipulation)
  (:domain robosuite)
  (:language pick up the black bowl and place it on the plate)
  (:regions
    (plate_region
      (:target main_table)
      (:ranges (
        (0.05 0.19 0.07 0.21)
      ))
    )
    (bowl_region
      (:target main_table)
      (:ranges (
        (-0.1 -0.01 -0.05 0.01)
      ))
    )
  )
  (:fixtures
    main_table - table
  )
  (:objects
    akita_black_bowl_1 - akita_black_bowl
    plate_1 - plate
  )
  (:obj_of_interest
    akita_black_bowl_1
    plate_1
  )
  (:init
    (On akita_black_bowl_1 main_table_bowl_region)
    (On plate_1 main_table_plate_region)
  )
  (:goal
    (And (On akita_black_bowl_1 plate_1))
  )
)";

        [Test]
        public void Parse_ValidSample_CorrectProblemName()
        {
            var problem = BDDLParser.Parse(SampleBDDL);

            Assert.AreEqual("libero_tabletop_manipulation", problem.ProblemName,
                "Problem name should be lowercased from BDDL");
        }

        [Test]
        public void Parse_ValidSample_CorrectDomain()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            Assert.AreEqual("robosuite", problem.DomainName);
        }

        [Test]
        public void Parse_ValidSample_LanguageInstruction()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            StringAssert.Contains("pick up the black bowl", problem.LanguageInstruction);
        }

        [Test]
        public void Parse_ValidSample_RegionsCount()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            Assert.AreEqual(2, problem.Regions.Count);
            Assert.IsTrue(problem.Regions.ContainsKey("main_table_plate_region"));
            Assert.IsTrue(problem.Regions.ContainsKey("main_table_bowl_region"));
        }

        [Test]
        public void Parse_ValidSample_RegionRanges()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            var plateRegion = problem.Regions["main_table_plate_region"];

            Assert.AreEqual("main_table", plateRegion.Target);
            Assert.AreEqual(1, plateRegion.Ranges.Count);

            var range = plateRegion.Ranges[0];
            Assert.AreEqual(0.05f, range.XMin, 0.001f);
            Assert.AreEqual(0.19f, range.YMin, 0.001f);
            Assert.AreEqual(0.07f, range.XMax, 0.001f);
            Assert.AreEqual(0.21f, range.YMax, 0.001f);
        }

        [Test]
        public void Parse_ValidSample_Fixtures()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            Assert.AreEqual(1, problem.Fixtures.Count);
            Assert.AreEqual("table", problem.Fixtures["main_table"]);
        }

        [Test]
        public void Parse_ValidSample_Objects()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            Assert.AreEqual(2, problem.Objects.Count);
            Assert.AreEqual("akita_black_bowl", problem.Objects["akita_black_bowl_1"]);
            Assert.AreEqual("plate", problem.Objects["plate_1"]);
        }

        [Test]
        public void Parse_ValidSample_ObjOfInterest()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            Assert.AreEqual(2, problem.ObjOfInterest.Count);
            CollectionAssert.Contains(problem.ObjOfInterest, "akita_black_bowl_1");
            CollectionAssert.Contains(problem.ObjOfInterest, "plate_1");
        }

        [Test]
        public void Parse_ValidSample_InitStates()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            Assert.AreEqual(2, problem.InitialState.Count);

            var s0 = problem.InitialState[0];
            Assert.AreEqual(InitPredicateType.On, s0.Predicate);
            Assert.AreEqual("akita_black_bowl_1", s0.ObjectName);
            Assert.AreEqual("main_table_bowl_region", s0.LocationName);

            var s1 = problem.InitialState[1];
            Assert.AreEqual(InitPredicateType.On, s1.Predicate);
            Assert.AreEqual("plate_1", s1.ObjectName);
        }

        [Test]
        public void Parse_ValidSample_Goal()
        {
            var problem = BDDLParser.Parse(SampleBDDL);
            Assert.AreEqual(1, problem.Goal.Conditions.Count);

            var cond = problem.Goal.Conditions[0];
            Assert.AreEqual(GoalType.On, cond.Type);
            Assert.AreEqual("akita_black_bowl_1", cond.ObjectName);
            Assert.AreEqual("plate_1", cond.TargetName);
        }
    }

    public class RegionSamplerTests
    {
        [Test]
        public void SamplePosition_ReturnsPositionWithinRanges()
        {
            var region = new BDDLRegion("test")
            {
                Target = "main_table",
                Ranges = { new RegionRange(-1f, -1f, 1f, 1f) }
            };

            for (int i = 0; i < 100; i++)
            {
                float x = RegionSampler.SampleX(region);
                float y = RegionSampler.SampleY(region);

                Assert.GreaterOrEqual(x, -1f);
                Assert.LessOrEqual(x, 1f);
                Assert.GreaterOrEqual(y, -1f);
                Assert.LessOrEqual(y, 1f);
            }
        }
    }
}
