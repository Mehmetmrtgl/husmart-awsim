// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System.Linq;
using UnityEngine;

namespace Awsim.Common
{
    public class OsmToLaneletMap
    {
        Vector3 _origin;
        LaneletMap _map;

        public OsmToLaneletMap(Vector3 origin)
        {
            this._origin = origin;
        }

        public LaneletMap Convert(OsmData osm)
        {
            _map = new LaneletMap();
            LoadNodes(osm);
            LoadWays(osm);
            LoadLanelets(osm);
            LoadRegulatoryElements(osm);
            return _map;
        }

        void LoadNodes(OsmData osm)
        {
            foreach (var node in osm.Nodes)
            {
                var attributes = new LaneletAttribute(node.Tags);
                attributes.TryGetLocalPosition(out Vector3 localPosition);
                var position = Ros2Utility.Ros2ToUnityPosition(localPosition - _origin);
                _map.Add(new LaneletPointData(node.Id, attributes, position));
            }
        }

        void LoadWays(OsmData osm)
        {
            foreach (var way in osm.Ways)
            {
                var attributes = new LaneletAttribute(way.Tags);
                var result = attributes.TryGetValue(LaneletAttributeKey.Area, out bool isArea);
                if (result && isArea)
                {
                    continue;
                }
                var lineString = ExtractLineStringData(way);
                _map.Add(lineString);
            }
        }

        void LoadLanelets(OsmData osm)
        {
            foreach (var relation in osm.Relations)
            {
                var attributes = new LaneletAttribute(relation.Tags);
                if (attributes[LaneletAttributeKey.Type] != LaneletAttributeValue.Lanelet)
                {
                    continue;
                }
                var id = relation.Id;
                var leftBorder = GetLine(relation, "left");
                var rightBorder = GetLine(relation, "right");
                OrientBounds(ref leftBorder, ref rightBorder);
                var lanelet = new LaneletData(id, attributes, leftBorder, rightBorder);
                _map.Add(lanelet);
            }
        }

        // Beytepe: OSM'de sınır çizgileri (way) lanelet yönünün tersine kaydedilmiş olabilir. Lanelet2/Autoware
        // yönü geometriden (sol sınır solda kalacak şekilde) belirler; aynı kural burada uygulanır, yoksa
        // TrafficLane'ler ters yönde üretilir. Paylaşılan çizgi nesneleri değiştirilmez, ters kopya kullanılır.
        static Vector2 RosXY(Vector3 u) => new Vector2(u.z, -u.x);   // ROS düzlemi: x = unity.z, y = -unity.x

        static void OrientBounds(ref LaneletLineStringData left, ref LaneletLineStringData right)
        {
            if (left == null || right == null || left.Points.Length < 2 || right.Points.Length < 2)
                return;
            var L = left.Points; var R = right.Points;
            int nl = L.Length - 1, nr = R.Length - 1;
            float same = Vector2.Distance(RosXY(L[0]), RosXY(R[0])) + Vector2.Distance(RosXY(L[nl]), RosXY(R[nr]));
            float cross = Vector2.Distance(RosXY(L[0]), RosXY(R[nr])) + Vector2.Distance(RosXY(L[nl]), RosXY(R[0]));
            if (same > cross)
                right = Reversed(right);
            R = right.Points;
            var dir = (RosXY(L[nl]) + RosXY(R[nr]) - RosXY(L[0]) - RosXY(R[0])) * 0.5f;
            var side = RosXY(L[L.Length / 2]) - RosXY(R[R.Length / 2]);
            if (dir.x * side.y - dir.y * side.x < 0f)   // sol sınır sağda kalıyor -> ikisini de çevir
            {
                left = Reversed(left);
                right = Reversed(right);
            }
        }

        static LaneletLineStringData Reversed(LaneletLineStringData s)
            => new LaneletLineStringData(s.Id, s.Attributes, s.Points.Reverse().ToArray());

        void LoadRegulatoryElements(OsmData osm)
        {
            foreach (var relation in osm.Relations)
            {
                var attributes = new LaneletAttribute(relation.Tags);
                if (attributes[LaneletAttributeKey.Type] != LaneletAttributeValue.RegulatoryElement)
                    continue;

                var id = relation.Id;
                var subtype = attributes[LaneletAttributeKey.Subtype];

                switch (subtype)
                {
                    case LaneletAttributeValue.RightOfWay:
                        var yields = GetLanelets(relation, "yield");
                        var rightOfWays = GetLanelets(relation, "right_of_way");
                        var regElem = LaneletRegulatoryElement.CreateRightOfWay(
                            id, attributes, yields, rightOfWays);
                        _map.Add(regElem);
                        break;
                    case LaneletAttributeValue.TrafficSign:
                        var refLines = GetLines(relation, "ref_line");
                        var refers = GetLines(relation, "refers");
                        regElem = LaneletRegulatoryElement.CreateTrafficSign(
                            id, attributes, refLines, refers);
                        _map.Add(regElem);
                        break;
                    case LaneletAttributeValue.TrafficLight:
                        refLines = GetLines(relation, "ref_line");
                        refers = GetLines(relation, "refers");
                        var lightBulbs = GetLines(relation, "light_bulbs");
                        regElem = LaneletRegulatoryElement.CreateTrafficLight(
                            id, attributes, refLines, refers, lightBulbs);
                        _map.Add(regElem);
                        break;
                }
            }

            foreach (var relation in osm.Relations)
            {
                var attributes = new LaneletAttribute(relation.Tags);
                if (attributes[LaneletAttributeKey.Type] != LaneletAttributeValue.Lanelet)
                    continue;

                var id = relation.Id;
                var regulatoryElements = GetRegulatoryElements(relation);
                _map.Lanelets[id].RegulatoryElements = regulatoryElements;
            }
        }

        LaneletLineStringData ExtractLineStringData(Way way)
        {
            return new LaneletLineStringData(way.Id, new LaneletAttribute(way.Tags), way.NodeIds.Select(id => _map.Points[id]).ToArray());
        }

        LaneletLineStringData GetLine(Relation relation, string role)
        {
            var id = relation.Members.FirstOrDefault(member => member.Role == role).RefId;
            if (id == 0)
            {
                return null;
            }
            return _map.Lines[id];
        }

        LaneletLineStringData[] GetLines(Relation relation, string role)
        {
            return relation.Members
                .Where(member => member.Role == role)
                .Select(member => _map.Lines[member.RefId])
                .ToArray();
        }

        LaneletRegulatoryElement[] GetRegulatoryElements(Relation relation)
        {
            return relation.Members
                .Where(member => member.Role == "regulatory_element")
                .Where(member => _map.RegulatoryElements.ContainsKey(member.RefId))
                .Select(member => _map.RegulatoryElements[member.RefId])
                .ToArray();
        }

        LaneletData[] GetLanelets(Relation relation, string role)
        {
            return relation.Members
                .Where(member => member.Role == role)
                .Select(member => _map.Lanelets[member.RefId])
                .ToArray();
        }
    }
}
