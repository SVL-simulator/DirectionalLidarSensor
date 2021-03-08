/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using System;
using System.Threading.Tasks;
using System.Collections.Generic;
using Unity.Profiling;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Utilities;
using Simulator.PointCloud;
using Simulator.Sensors.UI;

namespace Simulator.Sensors
{
    [SensorType("DirectionalLidar", new[] { typeof(PointCloudData) })]
    public partial class DirectionalLidarSensor : SensorBase
    {
        private static class Properties
        {
            public static readonly int Input = Shader.PropertyToID("_Input");
            public static readonly int Output = Shader.PropertyToID("_Output");
            public static readonly int Index = Shader.PropertyToID("_Index");
            public static readonly int Count = Shader.PropertyToID("_Count");
            public static readonly int LaserCount = Shader.PropertyToID("_LaserCount");
            public static readonly int MeasuresPerRotation = Shader.PropertyToID("_MeasurementsPerRotation");
            public static readonly int Origin = Shader.PropertyToID("_Origin");
            public static readonly int Transform = Shader.PropertyToID("_Transform");
            public static readonly int ScaleDistance = Shader.PropertyToID("_ScaleDistance");
            public static readonly int TexSize = Shader.PropertyToID("_TexSize");
            public static readonly int DirLidarStart = Shader.PropertyToID("_DirLidarStart");
            public static readonly int DirLidarForward = Shader.PropertyToID("_DirLidarForward");
            public static readonly int DirLidarDeltaX = Shader.PropertyToID("_DirLidarDeltaX");
            public static readonly int DirLidarDeltaY = Shader.PropertyToID("_DirLidarDeltaY");
            public static readonly int DirLidarMaxRayCount = Shader.PropertyToID("_DirLidarMaxRayCount");
            public static readonly int DirLidarStartRay = Shader.PropertyToID("_DirLidarStartRay");
            public static readonly int CosAngle = Shader.PropertyToID("_CosAngle");
        }
        
        // Lidar x is forward, y is left, z is up
        public static readonly Matrix4x4 LidarTransform = new Matrix4x4(new Vector4(0, -1, 0, 0), new Vector4(0, 0, 1, 0), new Vector4(1, 0, 0, 0), Vector4.zero);

        int CurrentLaserCount;
        int CurrentMeasurementsPerRotation;
        float CurrentFieldOfView;
        float CurrentCenterAngle;
        float CurrentMinDistance;
        float CurrentMaxDistance;
        float CurrentHorizontalAngle;

        const float HorizontalAngleLimit = 15.0f;

        [SensorParameter]
        [Range(1, 128)]
        public int LaserCount = 32;

        [SensorParameter]
        [Range(0.01f, 1000f)]
        public float MinDistance = 0.5f; // meters

        [SensorParameter]
        [Range(0.01f, 2000f)]
        public float MaxDistance = 100.0f; // meters

        [SensorParameter]
        [Range(1, 30)]
        public float RotationFrequency = 5.0f; // Hz

        [SensorParameter]
        [Range(18, 6000)] // minmimum is 360/HorizontalAngleLimit
        public int MeasurementsPerRotation = 1500; // for each ray

        [SensorParameter]
        [Range(1.0f, 45.0f)]
        public float FieldOfView = 40.0f;

        [SensorParameter]
        [Range(-45.0f, 45.0f)]
        public float CenterAngle = 10.0f;

        [SensorParameter]
        public bool Compensated = true;

        public GameObject Top = null;

        [SensorParameter]
        [Range(1, 10)]
        public float PointSize = 2.0f;

        [SensorParameter]
        public Color PointColor = Color.red;

        [SensorParameter]
        [Range(0f, 360f)]
        public float ForwardAngle = 0f;

        private Quaternion VehicleForward;

        [SensorParameter]
        [Range(0f, 360f)]
        public float HorizontalAngle = 180f;

        private float CosHorizontalAngle = 0f;

        BridgeInstance Bridge;
        Publisher<PointCloudData> Publish;
        uint SendSequence;

        private Vector4[] Points;

        ComputeBuffer PointCloudBuffer;
        int PointCloudLayer;

        Material PointCloudMaterial;

        private Camera Camera;

        private SensorRenderTarget activeTarget;
        private ShaderTagId passId;
        private ComputeShader cs;

        struct ReadRequest
        {
            public SensorRenderTarget TextureSet;
            public int Index;
            public int Count;
            public int MaxRayCount;
            public int StartRay;
            public float CosAngle;

            public Vector3 Origin;
            public Vector3 Start;
            public Vector3 DeltaX;
            public Vector3 DeltaY;
            public Vector3 Forward;

            public Matrix4x4 Transform;

            public float AngleEnd;
        }

        List<ReadRequest> Active = new List<ReadRequest>();

        Stack<SensorRenderTarget> AvailableRenderTextures = new Stack<SensorRenderTarget>();
        Stack<Texture2D> AvailableTextures = new Stack<Texture2D>();

        int CurrentIndex;
        float AngleStart;
        float AngleDelta;

        float AngleTopPart;

        float MaxAngle;
        int RenderTextureWidth;
        int RenderTextureHeight;

        float FixupAngle;
        float IgnoreNewRquests;

        ProfilerMarker UpdateMarker = new ProfilerMarker("Lidar.Update");
        ProfilerMarker VisualizeMarker = new ProfilerMarker("Lidar.Visualzie");
        ProfilerMarker BeginReadMarker = new ProfilerMarker("Lidar.BeginRead");
        ProfilerMarker EndReadMarker = new ProfilerMarker("Lidar.EndRead");

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            Bridge = bridge;
            Publish = bridge.AddPublisher<PointCloudData>(Topic);
        }

        public void CustomRender(ScriptableRenderContext context, HDCamera hd)
        {
            var cmd = CommandBufferPool.Get();
            SensorPassRenderer.Render(context, cmd, hd, activeTarget, passId, Color.clear);
            PointCloudManager.RenderLidar(context, cmd, hd, activeTarget.ColorHandle, activeTarget.DepthHandle);
            CommandBufferPool.Release(cmd);
        }

        public void Init()
        {
            Camera = GetComponentInChildren<Camera>();
            Camera.GetComponent<HDAdditionalCameraData>().customRender += CustomRender;
            PointCloudMaterial = new Material(RuntimeSettings.Instance.PointCloudShader);
            PointCloudLayer = LayerMask.NameToLayer("Sensor Effects");
            passId = new ShaderTagId("SimulatorLidarPass");
            cs = Instantiate(RuntimeSettings.Instance.LidarComputeShader);
            
            Reset();
        }

        private void Start()
        {
            Init();
        }

        public void Reset()
        {
            Active.ForEach(req =>
            {
                req.TextureSet.Release();
            });
            Active.Clear();

            foreach (var tex in AvailableRenderTextures)
            {
                tex.Release();
            };
            AvailableRenderTextures.Clear();

            foreach (var tex in AvailableTextures)
            {
                Destroy(tex);
            };
            AvailableTextures.Clear();

            if (PointCloudBuffer != null)
            {
                PointCloudBuffer.Release();
                PointCloudBuffer = null;
            }

            FixupAngle = AngleStart = 0.0f;
            MaxAngle = Mathf.Abs(CenterAngle) + FieldOfView / 2.0f;
            RenderTextureHeight = 2 * (int)(2.0f * MaxAngle * LaserCount / FieldOfView);
            RenderTextureWidth = 2 * (int)(HorizontalAngleLimit / (360.0f / MeasurementsPerRotation));

            // construct custom aspect ratio projection matrix
            // math from https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix

            float v = 1.0f / Mathf.Tan(MaxAngle * Mathf.Deg2Rad);
            float h = 1.0f / Mathf.Tan(HorizontalAngleLimit * Mathf.Deg2Rad / 2.0f);
            float a = (MaxDistance + MinDistance) / (MinDistance - MaxDistance);
            float b = 2.0f * MaxDistance * MinDistance / (MinDistance - MaxDistance);

            var projection = new Matrix4x4(
                new Vector4(h, 0, 0, 0),
                new Vector4(0, v, 0, 0),
                new Vector4(0, 0, a, -1),
                new Vector4(0, 0, b, 0));

            Camera.nearClipPlane = MinDistance;
            Camera.farClipPlane = MaxDistance;
            Camera.projectionMatrix = projection;

            int count = LaserCount * MeasurementsPerRotation;
            PointCloudBuffer = new ComputeBuffer(count, UnsafeUtility.SizeOf<Vector4>());
            PointCloudMaterial.SetBuffer("_PointCloud", PointCloudBuffer);

            Points = new Vector4[count];

            CurrentLaserCount = LaserCount;
            CurrentMeasurementsPerRotation = MeasurementsPerRotation;
            CurrentFieldOfView = FieldOfView;
            CurrentCenterAngle = CenterAngle;
            CurrentMinDistance = MinDistance;
            CurrentMaxDistance = MaxDistance;
            CurrentHorizontalAngle = HorizontalAngle;

            CosHorizontalAngle = Mathf.Cos(Mathf.Deg2Rad * HorizontalAngle / 2f);

            IgnoreNewRquests = 0;
        }

        void OnDisable()
        {
            Active.ForEach(req =>
            {
                req.TextureSet.Release();
            });
            Active.Clear();
        }

        void Update()
        {
            if (LaserCount != CurrentLaserCount ||
                MeasurementsPerRotation != CurrentMeasurementsPerRotation ||
                FieldOfView != CurrentFieldOfView ||
                CenterAngle != CurrentCenterAngle ||
                MinDistance != CurrentMinDistance ||
                MaxDistance != CurrentMaxDistance ||
                HorizontalAngle != CurrentHorizontalAngle
                )
            {
                if (MinDistance > 0 && MaxDistance > 0 && LaserCount > 0 && MeasurementsPerRotation >= (360.0f / HorizontalAngleLimit))
                {
                    Reset();
                }
            }

            UpdateMarker.Begin();

            var cmd = CommandBufferPool.Get();
            foreach (var req in Active)
            {
                if (!req.TextureSet.IsValid(RenderTextureWidth, RenderTextureHeight))
                {
                    // lost render texture, probably due to Unity window resize or smth
                    req.TextureSet.Release();
                }
                else
                {
                    EndReadRequest(cmd, req);
                    AvailableRenderTextures.Push(req.TextureSet);

                    if (req.Index + req.Count >= CurrentMeasurementsPerRotation)
                    {
                        SendMessage();
                    }
                }
            }

            HDRPUtilities.ExecuteAndClearCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);

            Active.Clear();

            if (IgnoreNewRquests > 0)
            {
                IgnoreNewRquests -= Time.unscaledDeltaTime;
            }
            else
            {
                float minAngle = 360.0f / CurrentMeasurementsPerRotation;

                AngleDelta += Time.deltaTime * 360.0f * RotationFrequency;
                int count = (int)(HorizontalAngleLimit / minAngle);

                while (AngleDelta >= HorizontalAngleLimit)
                {
                    float angle = AngleStart + HorizontalAngleLimit / 2.0f;
                    var rotation = Quaternion.AngleAxis(angle, Vector3.up);
                    Camera.transform.localRotation = rotation;
                    if (Top != null)
                    {
                        Top.transform.localRotation = rotation;
                    }

                    var req = new ReadRequest();
                    if (BeginReadRequest(count, AngleStart, HorizontalAngleLimit, ref req))
                    {
                        Active.Add(req);
                    }

                    AngleDelta -= HorizontalAngleLimit;
                    AngleStart += HorizontalAngleLimit;

                    if (AngleStart >= 360.0f)
                    {
                        AngleStart -= 360.0f;
                    }
                }
            }

            UpdateMarker.End();
        }

        public void OnDestroy()
        {
            Active.ForEach(req =>
            {
                req.TextureSet.Release();
            });

            foreach (var tex in AvailableRenderTextures)
            {
                tex.Release();
            }
            foreach (var tex in AvailableTextures)
            {
                DestroyImmediate(tex);
            }

            PointCloudBuffer.Release();

            if (PointCloudMaterial != null)
            {
                DestroyImmediate(PointCloudMaterial);
            }
        }

        bool BeginReadRequest(int count, float angleStart, float angleUse, ref ReadRequest req)
        {
            if (count == 0)
            {
                return false;
            }

            BeginReadMarker.Begin();

            SensorRenderTarget renderTarget = null;
            if (AvailableRenderTextures.Count != 0)
                renderTarget = AvailableRenderTextures.Pop();

            if (renderTarget == null)
            {
                renderTarget = SensorRenderTarget.Create2D(RenderTextureWidth, RenderTextureHeight);
            }
            else if (!renderTarget.IsValid(RenderTextureWidth, RenderTextureHeight))
            {
                renderTarget.Release();
                renderTarget = SensorRenderTarget.Create2D(RenderTextureWidth, RenderTextureHeight);
            }

            activeTarget = renderTarget;
            Camera.targetTexture = renderTarget;
            Camera.Render();

            var pos = Camera.transform.position;

            var topLeft = Camera.ViewportPointToRay(new Vector3(0, 0, 1)).direction;
            var topRight = Camera.ViewportPointToRay(new Vector3(1, 0, 1)).direction;
            var bottomLeft = Camera.ViewportPointToRay(new Vector3(0, 1, 1)).direction;
            var bottomRight = Camera.ViewportPointToRay(new Vector3(1, 1, 1)).direction;

            int maxRayCount = (int)(2.0f * MaxAngle * LaserCount / FieldOfView);
            var deltaX = (topRight - topLeft) / count;
            var deltaY = (bottomLeft - topLeft) / maxRayCount;

            int startRay = 0;
            var start = topLeft;
            if (CenterAngle < 0.0f)
            {
                startRay = maxRayCount - LaserCount;
            }

            req = new ReadRequest()
            {
                TextureSet = renderTarget,
                Index = CurrentIndex,
                Count = count,
                MaxRayCount = maxRayCount,
                StartRay = startRay,
                Origin = pos,
                Start = start,
                DeltaX = deltaX,
                DeltaY = deltaY,
                AngleEnd = angleStart + angleUse,
                Forward = Quaternion.Euler(0, ForwardAngle, 0) * transform.forward,
                CosAngle = CosHorizontalAngle,
            };

            if (!Compensated)
            {
                req.Transform = transform.worldToLocalMatrix;
            }

            BeginReadMarker.End();

            CurrentIndex = (CurrentIndex + count) % CurrentMeasurementsPerRotation;

            return true;
        }

        private void EndReadRequest(CommandBuffer cmd, ReadRequest req)
        {
            EndReadMarker.Begin();

            var kernel = cs.FindKernel(Compensated ? "DirLidarComputeComp" : "DirLidarCompute");
            cmd.SetComputeTextureParam(cs, kernel, Properties.Input, req.TextureSet.ColorTexture);
            cmd.SetComputeBufferParam(cs, kernel, Properties.Output, PointCloudBuffer);
            cmd.SetComputeIntParam(cs, Properties.Index, req.Index);
            cmd.SetComputeIntParam(cs, Properties.Count, req.Count);
            cmd.SetComputeIntParam(cs, Properties.LaserCount, CurrentLaserCount);
            cmd.SetComputeIntParam(cs, Properties.MeasuresPerRotation, CurrentMeasurementsPerRotation);
            cmd.SetComputeIntParam(cs, Properties.DirLidarMaxRayCount, req.MaxRayCount);
            cmd.SetComputeIntParam(cs, Properties.DirLidarStartRay, req.StartRay);
            cmd.SetComputeFloatParam(cs, Properties.CosAngle, req.CosAngle);
            cmd.SetComputeVectorParam(cs, Properties.Origin, req.Origin);
            cmd.SetComputeMatrixParam(cs, Properties.Transform, req.Transform);
            cmd.SetComputeVectorParam(cs, Properties.ScaleDistance, new Vector4(0f, 0f, MinDistance, MaxDistance));
            cmd.SetComputeVectorParam(cs, Properties.TexSize, new Vector4(RenderTextureWidth, RenderTextureHeight, 1f / RenderTextureWidth, 1f / RenderTextureHeight));
            cmd.SetComputeVectorParam(cs, Properties.DirLidarStart, req.Start);
            cmd.SetComputeVectorParam(cs, Properties.DirLidarForward, req.Forward);
            cmd.SetComputeVectorParam(cs, Properties.DirLidarDeltaX, req.DeltaX);
            cmd.SetComputeVectorParam(cs, Properties.DirLidarDeltaY, req.DeltaY);
            cmd.DispatchCompute(cs, kernel, HDRPUtilities.GetGroupSize(req.Count, 8), HDRPUtilities.GetGroupSize(LaserCount, 8), 1);

            EndReadMarker.End();
        }

        void SendMessage()
        {
            if (Bridge != null && Bridge.Status == Status.Connected)
            {
                var worldToLocal = LidarTransform;
                if (Compensated)
                {
                    worldToLocal = worldToLocal * transform.worldToLocalMatrix;
                }

                PointCloudBuffer.GetData(Points);

                Task.Run(() =>
                {
                    Publish(new PointCloudData()
                    {
                        Name = Name,
                        Frame = Frame,
                        Time = SimulatorManager.Instance.CurrentTime,
                        Sequence = SendSequence++,

                        LaserCount = CurrentLaserCount,
                        Transform = worldToLocal,
                        Points = Points,
                    });
                });
            }
        }

        public Vector4[] Capture()
        {
            Debug.Assert(Compensated); // points should be in world-space
            int rotationCount = Mathf.CeilToInt(360.0f / HorizontalAngleLimit);

            float minAngle = 360.0f / CurrentMeasurementsPerRotation;
            int count = (int)(HorizontalAngleLimit / minAngle);

            float angle = HorizontalAngleLimit / 2.0f;
            var textures = new Texture2D[rotationCount];

            var cmd = CommandBufferPool.Get();
            var rt = RenderTexture.active;

            try
            {
                for (int i = 0; i < rotationCount; i++)
                {
                    var rotation = Quaternion.AngleAxis(angle, Vector3.up);
                    Camera.transform.localRotation = rotation;

                    var req = new ReadRequest();
                    if (BeginReadRequest(count, angle, HorizontalAngleLimit, ref req))
                    {
                        RenderTexture.active = req.TextureSet;
                        Texture2D texture;
                        if (AvailableTextures.Count > 0)
                        {
                            texture = AvailableTextures.Pop();
                        }
                        else
                        {
                            texture = new Texture2D(RenderTextureWidth, RenderTextureHeight, TextureFormat.RGBA32, false, true);
                        }
                        texture.ReadPixels(new Rect(0, 0, RenderTextureWidth, RenderTextureHeight), 0, 0);
                        textures[i] = texture;
                        EndReadRequest(cmd, req);

                        AvailableRenderTextures.Push(req.TextureSet);
                    }

                    angle += HorizontalAngleLimit;
                    if (angle >= 360.0f)
                    {
                        angle -= 360.0f;
                    }
                }
            }
            finally
            {
                HDRPUtilities.ExecuteAndClearCommandBuffer(cmd);
                CommandBufferPool.Release(cmd);
                RenderTexture.active = rt;
                Array.ForEach(textures, AvailableTextures.Push);
            }

            PointCloudBuffer.GetData(Points);

            return Points;
        }

        public bool Save(string path)
        {
            int rotationCount = Mathf.CeilToInt(360.0f / HorizontalAngleLimit);

            float minAngle = 360.0f / CurrentMeasurementsPerRotation;
            int count = (int)(HorizontalAngleLimit / minAngle);

            float angle = HorizontalAngleLimit / 2.0f;

            var active = new ReadRequest[rotationCount];
            var cmd = CommandBufferPool.Get();

            try
            {
                for (int i = 0; i < rotationCount; i++)
                {
                    var rotation = Quaternion.AngleAxis(angle, Vector3.up);
                    Camera.transform.localRotation = rotation;

                    BeginReadRequest(count, angle, HorizontalAngleLimit, ref active[i]);

                    angle += HorizontalAngleLimit;
                    if (angle >= 360.0f)
                    {
                        angle -= 360.0f;
                    }
                }

                for (int i = 0; i < rotationCount; i++)
                {
                    EndReadRequest(cmd, active[i]);
                }
            }
            finally
            {
                HDRPUtilities.ExecuteAndClearCommandBuffer(cmd);
                CommandBufferPool.Release(cmd);
                Array.ForEach(active, req => AvailableRenderTextures.Push(req.TextureSet));
            }

            PointCloudBuffer.GetData(Points);

            var worldToLocal = LidarTransform;
            if (Compensated)
            {
                worldToLocal = worldToLocal * transform.worldToLocalMatrix;
            }

            try
            {
                using (var writer = new PcdWriter(path))
                {
                    for (int p = 0; p < Points.Length; p++)
                    {
                        var point = Points[p];
                        if (point != Vector4.zero)
                        {
                            writer.Write(worldToLocal.MultiplyPoint3x4(point), point.w);
                        }
                    };
                }

                return true;
            }
            catch (Exception ex)
            {
                Debug.LogException(ex);
                return false;
            }
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            VisualizeMarker.Begin();
            var lidarToWorld = Compensated ? Matrix4x4.identity : transform.localToWorldMatrix;
            PointCloudMaterial.SetMatrix("_LocalToWorld", lidarToWorld);
            PointCloudMaterial.SetFloat("_Size", PointSize * Utility.GetDpiScale());
            PointCloudMaterial.SetColor("_Color", PointColor);
            Graphics.DrawProcedural(PointCloudMaterial, new Bounds(transform.position, MaxDistance * Vector3.one), MeshTopology.Points, PointCloudBuffer.count, layer: LayerMask.NameToLayer("Sensor"));

            VisualizeMarker.End();
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }
    }
}
