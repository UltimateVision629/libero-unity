using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using LIBERO.Core;

namespace LIBERO.Tests
{
    public class LiberoEnvironmentTests
    {
        [Test]
        public void Environment_Initialize_WithValidBDDL()
        {
            var go = new GameObject("TestEnv");
            var env = go.AddComponent<LiberoEnvironment>();

            string testBDDL = @"
(define (problem LIBERO_Tabletop_Manipulation)
  (:domain robosuite)
  (:language test instruction)
  (:regions
    (plate_region
      (:target main_table)
      (:ranges ((0.05 0.19 0.07 0.21)))
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
    (On plate_1 main_table_plate_region)
  )
  (:goal
    (And (On akita_black_bowl_1 plate_1))
  )
)";

            var tempFile = System.IO.Path.Combine(Application.temporaryCachePath, "test.bddl");
            System.IO.File.WriteAllText(tempFile, testBDDL);

            env.Initialize(tempFile);

            Assert.IsNotNull(env.Problem);
            Assert.AreEqual("test instruction", env.GetLanguageInstruction());
            Assert.IsNotNull(env.Scene);
            Assert.AreEqual(1, env.Scene.FixtureStates.Count);
            Assert.AreEqual(2, env.Scene.ObjectStates.Count);

            Object.DestroyImmediate(go);
        }

        [Test]
        public void Environment_Reset_ReturnsValidObservation()
        {
            var go = new GameObject("TestEnv");
            var env = go.AddComponent<LiberoEnvironment>();
            var obsCollector = go.AddComponent<ObservationCollector>();

            string testBDDL = @"
(define (problem LIBERO_Tabletop_Manipulation)
  (:domain robosuite)
  (:language reset test)
  (:regions
    (bowl_region (:target main_table) (:ranges ((0.0 0.0 0.1 0.1))))
    (plate_region (:target main_table) (:ranges ((0.2 0.2 0.3 0.3))))
  )
  (:fixtures main_table - table)
  (:objects
    akita_black_bowl_1 - akita_black_bowl
    plate_1 - plate
  )
  (:obj_of_interest akita_black_bowl_1 plate_1)
  (:init
    (On akita_black_bowl_1 main_table_bowl_region)
    (On plate_1 main_table_plate_region)
  )
  (:goal (And (On akita_black_bowl_1 plate_1)))
)";

            var tempFile = System.IO.Path.Combine(Application.temporaryCachePath, "test_reset.bddl");
            System.IO.File.WriteAllText(tempFile, testBDDL);

            env.ObsCollector = obsCollector;
            env.Initialize(tempFile);

            var obs = env.ResetEnvironment();

            Assert.IsNotNull(obs.JointPositions);
            Assert.AreEqual(7, obs.JointPositions.Length);
            Assert.IsNotNull(obs.ObjectPositions);
            Assert.IsTrue(obs.ObjectPositions.ContainsKey("akita_black_bowl_1"));

            Object.DestroyImmediate(go);
        }

        [Test]
        public void Step_WithZeroAction_ReturnsObsRewardDone()
        {
            var go = new GameObject("TestEnv");
            var env = go.AddComponent<LiberoEnvironment>();
            var obsCollector = go.AddComponent<ObservationCollector>();

            string testBDDL = @"
(define (problem LIBERO_Tabletop_Manipulation)
  (:domain robosuite)
  (:language step test)
  (:regions
    (bowl_region (:target main_table) (:ranges ((0.0 0.0 0.1 0.1))))
  )
  (:fixtures main_table - table)
  (:objects akita_black_bowl_1 - akita_black_bowl)
  (:obj_of_interest akita_black_bowl_1)
  (:init (On akita_black_bowl_1 main_table_bowl_region))
  (:goal (And (On akita_black_bowl_1 main_table)))
)";

            var tempFile = System.IO.Path.Combine(Application.temporaryCachePath, "test_step.bddl");
            System.IO.File.WriteAllText(tempFile, testBDDL);

            env.ObsCollector = obsCollector;
            env.Initialize(tempFile);
            env.ResetEnvironment();

            Observation obs;
            float reward;
            bool done;
            Dictionary<string, object> info;
            (obs, reward, done, info) = env.Step(new float[] { 0, 0, 0, 0, 0, 0, 0 });

            Assert.IsNotNull(obs);
            Assert.IsTrue(reward >= 0f);
            Assert.IsNotNull(info);

            Object.DestroyImmediate(go);
        }
    }
}
