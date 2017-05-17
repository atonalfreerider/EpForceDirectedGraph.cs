/*! 
@file ForceDirected.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/epForceDirectedGraph.cs>
@date August 08, 2013
@brief ForceDirected Interface
@version 1.0

@section LICENSE

The MIT License (MIT)

Copyright (c) 2013 Woong Gyu La <juhgiyo@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

@section DESCRIPTION

An Interface for the ForceDirected Class.

*/
using System.Collections.Generic;
using EpForceDirectedGraph.cs;
using UnityEngine;

namespace Assets
{

    // intermediary struct for Compute Shader;
    public struct PointStruct
    {
        public Vector3 position;
        public Vector3 acceleration;
    }

    public struct SpringStruct
    {
        public bool connected;
        public float Length;
        public float K;
    }

    public abstract class ForceDirected<Vector> : IForceDirected where Vector : IVector
    {
        public float Stiffness
        {
            get;
            set;
        }

        public float Repulsion
        {
            get;
            set;
        }

        public float Damping
        {
            get;
            set;
        }

        public float Threadshold
        {
            get;
            set;
        }

        public bool WithinThreashold
        {
            get;
            private set;
        }
        protected Dictionary<string, Point> m_nodePoints;
        protected Dictionary<string, Spring> m_edgeSprings;
        public IGraph graph
        {
            get;
            protected set;
        }
        public void Clear()
        {
            m_nodePoints.Clear();
            m_edgeSprings.Clear();
            graph.Clear();
        }

        private int mComputeShaderKernelID;
        ComputeShader computeShader;
        ComputeBuffer readPointBuffer;
        ComputeBuffer assignPointBuffer;
        ComputeBuffer springBuffer;

        public ForceDirected(IGraph iGraph, float iStiffness, float iRepulsion, float iDamping, ComputeShader computeShader)
        {
            graph = iGraph;
            Stiffness = iStiffness;
            Repulsion = iRepulsion;
            Damping = iDamping;
            m_nodePoints = new Dictionary<string, Point>();
            m_edgeSprings = new Dictionary<string, Spring>();

            Threadshold = 0.01f;

            // generate points and springs;
            EachNode(delegate (Node n, Point point) { });
            EachEdge(delegate (Edge edge, Spring spring) { });

            this.computeShader = computeShader;

            // Create the ComputeBuffer holding the Nodes
            readPointBuffer = new ComputeBuffer(m_nodePoints.Values.Count, sizeof(float) * 3 * 2);
            assignPointBuffer = new ComputeBuffer(m_nodePoints.Values.Count, sizeof(float) * 3 * 2);
            SetPointBufferData(assignPointBuffer);

            springBuffer = new ComputeBuffer(m_nodePoints.Values.Count * m_nodePoints.Values.Count, sizeof(float) * 3);
            SetSpringBufferData();

            // Find the id of the kernel
            mComputeShaderKernelID = computeShader.FindKernel("ComputeForces");

            // Bind the ComputeBuffer to the compute shader
            computeShader.SetBuffer(mComputeShaderKernelID, "readPointBuffer", readPointBuffer);
            computeShader.SetBuffer(mComputeShaderKernelID, "assignPointBuffer", assignPointBuffer);
            computeShader.SetBuffer(mComputeShaderKernelID, "springBuffer", springBuffer);
            computeShader.SetFloat("Repulsion", Repulsion);
            computeShader.SetFloat("Stiffness", Stiffness);
            computeShader.SetFloat("CenterAttraction", 2f);
            computeShader.SetInt("numPoints", m_nodePoints.Values.Count);

        }

        void SetPointBufferData(ComputeBuffer setPointBuffer)
        {
            // copy positions and accelerations of nodes into intermediary array for compute buffer;
            PointStruct[] pointArr = new PointStruct[m_nodePoints.Values.Count];
            int count = 0;
            foreach (Point point in m_nodePoints.Values)
            {
                pointArr[count].position = new Vector3(point.position.x, point.position.y, point.position.z);
                pointArr[count].acceleration = new Vector3(point.acceleration.x, point.acceleration.y, point.acceleration.z);
                count++;
            }

            setPointBuffer.SetData(pointArr);

        }

        void SetSpringBufferData()
        {
            List<Point> transfer = new List<Point>();
            foreach (Point p in m_nodePoints.Values)
            {
                transfer.Add(p);
            }

            SpringStruct[] springs = new SpringStruct[m_nodePoints.Values.Count * m_nodePoints.Values.Count];
            foreach (Edge e in graph.edges)
            {
                Spring spring = GetSpring(e);
                springs[transfer.IndexOf(spring.point1) + transfer.IndexOf(spring.point2) * m_nodePoints.Values.Count] = new SpringStruct()
                {
                    connected = true,
                    Length = spring.Length,
                    K = spring.K
                };
            }

            springBuffer.SetData(springs);
        }

        public abstract Point GetPoint(Node iNode);

        public Spring GetSpring(Edge iEdge)
        {
            if (!(m_edgeSprings.ContainsKey(iEdge.ID)))
            {
                float length = iEdge.Data.length;
                Spring existingSpring = null;

                List<Edge> fromEdges = graph.GetEdges(iEdge.Source, iEdge.Target);
                if (fromEdges != null)
                {
                    foreach (Edge e in fromEdges)
                    {
                        if (existingSpring == null && m_edgeSprings.ContainsKey(e.ID))
                        {
                            existingSpring = m_edgeSprings[e.ID];
                            break;
                        }
                    }

                }
                if (existingSpring != null)
                {
                    return new Spring(existingSpring.point1, existingSpring.point2, 0.0f, 0.0f);
                }

                List<Edge> toEdges = graph.GetEdges(iEdge.Target, iEdge.Source);
                if (toEdges != null)
                {
                    foreach (Edge e in toEdges)
                    {
                        if (existingSpring == null && m_edgeSprings.ContainsKey(e.ID))
                        {
                            existingSpring = m_edgeSprings[e.ID];
                            break;
                        }
                    }
                }

                if (existingSpring != null)
                {
                    return new Spring(existingSpring.point2, existingSpring.point1, 0.0f, 0.0f);
                }
                m_edgeSprings[iEdge.ID] = new Spring(GetPoint(iEdge.Source), GetPoint(iEdge.Target), length, Stiffness);

            }
            return m_edgeSprings[iEdge.ID];
        }

        public void DisposeBuffers()
        {
            if (readPointBuffer != null)
                readPointBuffer.Dispose();
            if (assignPointBuffer != null)
                assignPointBuffer.Dispose();
            if (springBuffer != null)
                springBuffer.Dispose();
        }

        protected void updateVelocity(float iTimeStep)
        {
            foreach (Node n in graph.nodes)
            {
                Point point = GetPoint(n);
                point.velocity.Add(point.acceleration * iTimeStep);
                point.velocity.Multiply(Damping);
                Damping *= .9999f;
                point.acceleration.SetZero();
            }
        }

        protected void updatePosition(float iTimeStep)
        {
            foreach (Node n in graph.nodes)
            {
                Point point = GetPoint(n);
                point.position.Add(point.velocity * iTimeStep);
            }
        }

        protected float getTotalEnergy()
        {
            float energy = 0.0f;
            foreach (Node n in graph.nodes)
            {
                Point point = GetPoint(n);
                float speed = point.velocity.Magnitude();
                energy += 0.5f * point.mass * speed * speed;
            }
            return energy;
        }

        public void Calculate(float iTimeStep) // time in second
        {
            SetPointBufferData(readPointBuffer);

            // Calculate the repulsive force between every node in the graph;
            computeShader.Dispatch(mComputeShaderKernelID, m_nodePoints.Values.Count, m_nodePoints.Values.Count, 1);

            // copy the output of the compute shader into the dictionary containing the points;
            PointStruct[] pointArr = new PointStruct[m_nodePoints.Values.Count];
            assignPointBuffer.GetData(pointArr);
            int count = 0;
            foreach (Point p in m_nodePoints.Values)
            {
                p.acceleration = new FDGVector3(pointArr[count].acceleration.x, pointArr[count].acceleration.y, pointArr[count].acceleration.z);
                count++;
            }

            updateVelocity(iTimeStep);
            updatePosition(iTimeStep);
            if (getTotalEnergy() < Threadshold)
            {
                WithinThreashold = true;
            }
            else
                WithinThreashold = false;
        }

        public void EachEdge(EdgeAction del)
        {
            foreach (Edge e in graph.edges)
            {
                del(e, GetSpring(e));
            }
        }

        public void EachNode(NodeAction del)
        {
            foreach (Node n in graph.nodes)
            {
                del(n, GetPoint(n));
            }
        }

        public NearestPoint Nearest(AbstractVector position)
        {
            NearestPoint min = new NearestPoint();
            foreach (Node n in graph.nodes)
            {
                Point point = GetPoint(n);
                float distance = (point.position - position).Magnitude();
                if (min.distance == null || distance < min.distance)
                {
                    min.node = n;
                    min.point = point;
                    min.distance = distance;
                }
            }
            return min;
        }

        public abstract BoundingBox GetBoundingBox();

    }

    public class ForceDirected2D : ForceDirected<FDGVector2>
    {
        public ForceDirected2D(IGraph iGraph, float iStiffness, float iRepulsion, float iDamping, ComputeShader computeShader)
            : base(iGraph, iStiffness, iRepulsion, iDamping, computeShader)
        {

        }

        public override Point GetPoint(Node iNode)
        {
            if (!(m_nodePoints.ContainsKey(iNode.ID)))
            {
                FDGVector2 iniPosition = iNode.Data.initialPostion as FDGVector2;
                if (iniPosition == null)
                    iniPosition = FDGVector2.Random() as FDGVector2;
                m_nodePoints[iNode.ID] = new Point(iniPosition, FDGVector2.Zero(), FDGVector2.Zero(), iNode);
            }
            return m_nodePoints[iNode.ID];
        }

        public override BoundingBox GetBoundingBox()
        {
            BoundingBox boundingBox = new BoundingBox();
            FDGVector2 bottomLeft = FDGVector2.Identity().Multiply(BoundingBox.defaultBB * -1.0f) as FDGVector2;
            FDGVector2 topRight = FDGVector2.Identity().Multiply(BoundingBox.defaultBB) as FDGVector2;
            foreach (Node n in graph.nodes)
            {
                FDGVector2 position = GetPoint(n).position as FDGVector2;

                if (position.x < bottomLeft.x)
                    bottomLeft.x = position.x;
                if (position.y < bottomLeft.y)
                    bottomLeft.y = position.y;
                if (position.x > topRight.x)
                    topRight.x = position.x;
                if (position.y > topRight.y)
                    topRight.y = position.y;
            }
            AbstractVector padding = (topRight - bottomLeft).Multiply(BoundingBox.defaultPadding);
            boundingBox.bottomLeftFront = bottomLeft.Subtract(padding);
            boundingBox.topRightBack = topRight.Add(padding);
            return boundingBox;

        }
    }

    public class ForceDirected3D : ForceDirected<FDGVector3>
    {
        public ForceDirected3D(IGraph iGraph, float iStiffness, float iRepulsion, float iDamping, ComputeShader computeShader)
            : base(iGraph, iStiffness, iRepulsion, iDamping, computeShader)
        {

        }

        public override Point GetPoint(Node iNode)
        {
            if (!(m_nodePoints.ContainsKey(iNode.ID)))
            {
                FDGVector3 iniPosition = iNode.Data.initialPostion as FDGVector3;
                if (iniPosition == null)
                    iniPosition = FDGVector3.Random() as FDGVector3;
                m_nodePoints[iNode.ID] = new Point(iniPosition, FDGVector3.Zero(), FDGVector3.Zero(), iNode);
            }
            return m_nodePoints[iNode.ID];
        }

        public override BoundingBox GetBoundingBox()
        {
            BoundingBox boundingBox = new BoundingBox();
            FDGVector3 bottomLeft = FDGVector3.Identity().Multiply(BoundingBox.defaultBB * -1.0f) as FDGVector3;
            FDGVector3 topRight = FDGVector3.Identity().Multiply(BoundingBox.defaultBB) as FDGVector3;
            foreach (Node n in graph.nodes)
            {
                FDGVector3 position = GetPoint(n).position as FDGVector3;
                if (position.x < bottomLeft.x)
                    bottomLeft.x = position.x;
                if (position.y < bottomLeft.y)
                    bottomLeft.y = position.y;
                if (position.z < bottomLeft.z)
                    bottomLeft.z = position.z;
                if (position.x > topRight.x)
                    topRight.x = position.x;
                if (position.y > topRight.y)
                    topRight.y = position.y;
                if (position.z > topRight.z)
                    topRight.z = position.z;
            }
            AbstractVector padding = (topRight - bottomLeft).Multiply(BoundingBox.defaultPadding);
            boundingBox.bottomLeftFront = bottomLeft.Subtract(padding);
            boundingBox.topRightBack = topRight.Add(padding);
            return boundingBox;

        }
    }
}

