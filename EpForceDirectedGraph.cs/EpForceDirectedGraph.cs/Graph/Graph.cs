/*! 
@file Graph.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/epForceDirectedGraph.cs>
@date August 08, 2013
@brief Graph Interface
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

An Interface for the Graph Class.

*/
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace EpForceDirectedGraph.cs
{
    public class Graph: IGraph
    {
        public Graph()
        {
            m_nodeSet = new Dictionary<string, Node>();
            nodes = new List<Node>();
            edges = new List<Edge>();
            m_eventListeners = new List<IGraphEventListener>();
            m_adjacencySet = new Dictionary<string, Dictionary<string, List<Edge>>>();
        }
        public void Clear()
        {
            nodes.Clear();
            edges.Clear();
            m_adjacencySet.Clear();
        }
        public Node AddNode(Node iNode)
        {
            if (!m_nodeSet.ContainsKey(iNode.ID))
            {
                nodes.Add(iNode);
            }

            m_nodeSet[iNode.ID] = iNode;
            notify();
            return iNode;
        }

        public Edge AddEdge(Edge iEdge)
        {
            if (!edges.Contains(iEdge))
            {
                edges.Add(iEdge);
            }

            if (!(m_adjacencySet.ContainsKey(iEdge.Source.ID)))
            {
                m_adjacencySet[iEdge.Source.ID] = new Dictionary<string, List<Edge>>();
            }
            if (!(m_adjacencySet[iEdge.Source.ID].ContainsKey(iEdge.Target.ID)))
            {
                m_adjacencySet[iEdge.Source.ID][iEdge.Target.ID] = new List<Edge>();
            }

            if (!m_adjacencySet[iEdge.Source.ID][iEdge.Target.ID].Contains(iEdge))
            {
                m_adjacencySet[iEdge.Source.ID][iEdge.Target.ID].Add(iEdge);
            }

            notify();
            return iEdge;
        }
        public void CreateNodes(List<NodeData> iDataList)
        {
            foreach (NodeData t in iDataList)
            {
                CreateNode(t);
            }
        }

        public void CreateNodes(List<string> iNameList)
        {
            foreach (string t in iNameList)
            {
                CreateNode(t);
            }
        }

        public void CreateEdges(List<Triple<string, string, EdgeData>> iDataList)
        {
            foreach (Triple<string, string, EdgeData> t in iDataList)
            {
                if (!m_nodeSet.ContainsKey(t.first)) return;
                if (!m_nodeSet.ContainsKey(t.second)) return;
                Node node1 = m_nodeSet[t.first];
                Node node2 = m_nodeSet[t.second];
                CreateEdge(node1, node2, t.third);
            }
        }

        public void CreateEdges(List<Pair<string, string>> iDataList)
        {
            foreach (Pair<string, string> t in iDataList)
            {
                if (!m_nodeSet.ContainsKey(t.first)) return;
                if (!m_nodeSet.ContainsKey(t.second)) return;
                Node node1 = m_nodeSet[t.first];
                Node node2 = m_nodeSet[t.second];
                CreateEdge(node1, node2);
            }
        }

        public Node CreateNode(NodeData data)
        {
            Node tNewNode = new Node(m_nextNodeId.ToString(), data);
            m_nextNodeId++;
            AddNode(tNewNode);
            return tNewNode;
        }

        public Node CreateNode(string label)
        {
            NodeData data = new NodeData {label = label};
            Node tNewNode = new Node(m_nextNodeId.ToString(), data);
            m_nextNodeId++;
            AddNode(tNewNode);
            return tNewNode;
        }

        public Edge CreateEdge(Node iSource, Node iTarget, EdgeData iData = null)
        {
            if (iSource == null || iTarget == null) return null;

            Edge tNewEdge = new Edge(m_nextEdgeId.ToString(), iSource, iTarget, iData);
            m_nextEdgeId++;
            AddEdge(tNewEdge);
            return tNewEdge;
        }

        public Edge CreateEdge(string iSource, string iTarget, EdgeData iData = null)
        {
            if (!m_nodeSet.ContainsKey(iSource)) return null;
            if (!m_nodeSet.ContainsKey(iTarget)) return null;
            Node node1 = m_nodeSet[iSource];
            Node node2 = m_nodeSet[iTarget];
            return CreateEdge(node1, node2, iData);
        }


        public List<Edge> GetEdges(Node iNode1, Node iNode2)
        {
            if (m_adjacencySet.ContainsKey(iNode1.ID) &&
                m_adjacencySet[iNode1.ID].ContainsKey(iNode2.ID))
            {
                return m_adjacencySet[iNode1.ID][iNode2.ID];
            }
            return null;
        }

        public List<Edge> GetEdges(Node iNode)
        {
            List<Edge> retEdgeList = new List<Edge>();
            if (m_adjacencySet.ContainsKey(iNode.ID))
            {
                retEdgeList.AddRange(m_adjacencySet[iNode.ID].SelectMany(keyPair => keyPair.Value));
            }

            retEdgeList.AddRange(from keyValuePair in m_adjacencySet 
                where keyValuePair.Key != iNode.ID 
                from keyPair in m_adjacencySet[keyValuePair.Key] 
                from e in keyPair.Value 
                select e);
            return retEdgeList;
        }

        public void RemoveNode(Node iNode)
        {
            if (m_nodeSet.ContainsKey(iNode.ID))
            {
                m_nodeSet.Remove(iNode.ID);
            }
            nodes.Remove(iNode);
            DetachNode(iNode);

        }
        public void DetachNode(Node iNode)
        {
            edges.ForEach(delegate(Edge e)
            {
                if (e.Source.ID == iNode.ID || e.Target.ID == iNode.ID)
                {
                    RemoveEdge(e);
                }
            });
            notify();
        }

        public void RemoveEdge(Edge iEdge)
        {
            edges.Remove(iEdge);
            foreach (KeyValuePair<string, Dictionary<string, List<Edge>>> x in m_adjacencySet)
            {
                foreach (KeyValuePair<string, List<Edge>> y in x.Value)
                {
                    List<Edge> tEdges = y.Value;
                    tEdges.Remove(iEdge);
                    if (tEdges.Count != 0) continue;
                    m_adjacencySet[x.Key].Remove(y.Key);
                    break;
                }

                if (x.Value.Count != 0) continue;
                m_adjacencySet.Remove(x.Key);
                break;
            }
            notify();

        }

        public Node GetNode(string label)
        {
            Node retNode = null;
            nodes.ForEach(delegate(Node n)
            {
                if (n.Data.label == label)
                {
                    retNode = n;
                }
            });
            return retNode;
        }

        public Edge GetEdge(string label)
        {
            Edge retEdge = null;
            edges.ForEach(delegate(Edge e)
            {
                if (e.Data.label == label)
                {
                    retEdge = e;
                }
            });
            return retEdge;
        }
        public void Merge(Graph iMergeGraph)
        {
            foreach (Node n in iMergeGraph.nodes)
            {
                Node mergeNode = new Node(m_nextNodeId.ToString(), n.Data);
                AddNode(mergeNode);
                m_nextNodeId++;
                mergeNode.Data.origID=n.ID;
            }

            foreach (Edge unused in 
                from e in iMergeGraph.edges 
                let fromNode = nodes.Find(n => e.Source.ID == n.Data.origID) 
                let toNode = nodes.Find(n =>
                    e.Target.ID == n.Data.origID) 
                select AddEdge(new Edge(m_nextEdgeId.ToString(), fromNode, toNode, e.Data)))
            {
                m_nextEdgeId++;
            }
        }

        public void FilterNodes(Predicate<Node> match)
        {
            foreach (Node n in nodes.Where(n => !match(n)))
            {
                RemoveNode(n);
            }
        }

        public void FilterEdges(Predicate<Edge> match)
        {
            foreach (Edge e in edges.Where(e => !match(e)))
            {
                RemoveEdge(e);
            }
        }

        public void AddGraphListener(IGraphEventListener iListener)
        {
            m_eventListeners.Add(iListener);
        }

        void notify()
        {
            foreach (IGraphEventListener listener in m_eventListeners)
            {
                listener.GraphChanged();
            }
        }
        
        public List<Node> nodes
        {
            get;
            private set;
        }
        public List<Edge> edges
        {
            get;
            private set;
        }

        readonly Dictionary<string, Node> m_nodeSet;
        readonly Dictionary<string, Dictionary<string, List<Edge>>> m_adjacencySet;

        int m_nextNodeId = 0;
        int m_nextEdgeId = 0;
        readonly List<IGraphEventListener> m_eventListeners;
    }
}
