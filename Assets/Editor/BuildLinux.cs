using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using Mujoco;

/// <summary>
/// Linux standalone 构建入口（命令行触发）:
///   1) 校验 LinuxStandaloneSupport 模块存在
///   2) 新建空场景（DefaultGameObjects: Main Camera + Directional Light，缺光则 agentview 全黑）
///   3) 用官方 MjImporterWithAssets 导入 libero_put_block_in_box.xml（MjComponent 层级入当前场景；
///      MjScene 运行期懒创建 + FindObjectsOfType 收集，无需手动挂载）
///   4) 保存 scene → BuildPipeline.BuildPlayer(StandaloneLinux64)
/// 通过 BuildPlayerOptions.scenes 直接指定场景，不改动 EditorBuildSettings.asset。
/// </summary>
public static class BuildLinux
{
    const string XmlAsset =
        "Assets/LIBERO/assets/robots/so100_mjcf/libero_put_block_in_box.xml";
    const string SceneAsset =
        "Assets/Local/Build/libero_put_block_in_box.build.scene";
    const string OutDir = "Build/Linux";

    public static void Build()
    {
        string pe = Path.Combine(EditorApplication.applicationContentsPath,
            "PlaybackEngines", "LinuxStandaloneSupport");
        if (!Directory.Exists(pe))
            throw new System.Exception(
                "[BuildLinux] LinuxStandaloneSupport NOT installed: " + pe +
                " — 请在 Tuanjie Hub 为 2022.3.61t10 安装 'Linux 构建支持' 后重跑。");

        if (!AssetDatabase.IsValidFolder("Assets/Local/Build"))
        {
            if (!AssetDatabase.IsValidFolder("Assets/Local"))
                AssetDatabase.CreateFolder("Assets", "Local");
            AssetDatabase.CreateFolder("Assets/Local", "Build");
        }

        var scene = EditorSceneManager.NewScene(
            NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
        Debug.Log("[BuildLinux] NewScene created, importing " + XmlAsset);

        var importer = new MjImporterWithAssets();
        var root = importer.ImportFile(XmlAsset);
        if (root == null)
            throw new System.Exception("[BuildLinux] ImportFile returned null: " + XmlAsset);
        Debug.Log("[BuildLinux] Imported root: " + root.name + " at " + root.transform.position);

        if (!EditorSceneManager.SaveScene(scene, SceneAsset, true))
            throw new System.Exception("[BuildLinux] SaveScene failed: " + SceneAsset);

        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Standalone,
            ScriptingImplementation.Mono2x);

        // Vulkan 优先(服务器 NVIDIA GPU 硬件渲染,Vulkan 走 render 节点不依赖 Xorg/X11 GLX),
        // OpenGLCore 兜底(本地 Windows GPU / 无 Vulkan 环境自动回退)。
        PlayerSettings.SetGraphicsAPIs(
            BuildTarget.StandaloneLinux64,
            new[] { GraphicsDeviceType.Vulkan, GraphicsDeviceType.OpenGLCore });
        Debug.Log("[BuildLinux] GraphicsAPIs = "
                  + string.Join(", ", PlayerSettings.GetGraphicsAPIs(
                      BuildTarget.StandaloneLinux64)));

        var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = new[] { SceneAsset },
            locationPathName = Path.Combine(OutDir, "libero-unity"),
            target = BuildTarget.StandaloneLinux64,
            options = BuildOptions.None
        });
        var s = report.summary;
        Debug.Log($"[BuildLinux] result={s.result} errors={s.totalErrors} " +
                  $"warnings={s.totalWarnings} size={s.totalSize} out={s.outputPath} " +
                  $"time={report.summary.totalTime.TotalMinutes:F1}min");
        if (s.totalErrors > 0 || s.result != BuildResult.Succeeded)
            throw new System.Exception("[BuildLinux] BuildPlayer failed: " + s.result);
    }
}
