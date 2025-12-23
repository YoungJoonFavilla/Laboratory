using System;
using System.Collections.Generic;
using FixedMathSharp;
using UnityEngine;

namespace NavMesh2D.Geometry
{
    /// <summary>
    /// 2D 폴리곤 (정점 리스트로 정의)
    /// 시계 방향(CW) 또는 반시계 방향(CCW) 정점 순서 지원
    /// </summary>
    [Serializable]
    public class Polygon2D
    {
        [SerializeField]
        private List<Vector2Fixed> _vertices = new List<Vector2Fixed>();

        /// <summary>
        /// 정점 목록 (읽기 전용)
        /// </summary>
        public IReadOnlyList<Vector2Fixed> Vertices => _vertices;

        /// <summary>
        /// 정점 개수
        /// </summary>
        public int VertexCount => _vertices.Count;

        /// <summary>
        /// 유효한 폴리곤인지 (최소 3개 정점)
        /// </summary>
        public bool IsValid => _vertices.Count >= 3;

        public Polygon2D() { }

        public Polygon2D(IEnumerable<Vector2Fixed> vertices)
        {
            _vertices = new List<Vector2Fixed>(vertices);
        }

        public Polygon2D(params Vector2Fixed[] vertices)
        {
            _vertices = new List<Vector2Fixed>(vertices);
        }

        /// <summary>
        /// 정점 추가
        /// </summary>
        public void AddVertex(Vector2Fixed vertex)
        {
            _vertices.Add(vertex);
        }

        /// <summary>
        /// 정점 삽입
        /// </summary>
        public void InsertVertex(int index, Vector2Fixed vertex)
        {
            _vertices.Insert(index, vertex);
        }

        /// <summary>
        /// 정점 제거
        /// </summary>
        public void RemoveVertex(int index)
        {
            if (index >= 0 && index < _vertices.Count)
                _vertices.RemoveAt(index);
        }

        /// <summary>
        /// 정점 설정
        /// </summary>
        public void SetVertex(int index, Vector2Fixed vertex)
        {
            if (index >= 0 && index < _vertices.Count)
                _vertices[index] = vertex;
        }

        /// <summary>
        /// 정점 가져오기
        /// </summary>
        public Vector2Fixed GetVertex(int index)
        {
            // 순환 인덱스 지원
            int count = _vertices.Count;
            index = ((index % count) + count) % count;
            return _vertices[index];
        }

        /// <summary>
        /// 모든 정점 제거
        /// </summary>
        public void Clear()
        {
            _vertices.Clear();
        }

        /// <summary>
        /// 부호 있는 면적 계산 (Shoelace formula)
        /// 양수: 반시계 방향 (CCW)
        /// 음수: 시계 방향 (CW)
        /// </summary>
        public Fixed64 SignedArea()
        {
            if (_vertices.Count < 3)
                return Fixed64.Zero;

            Fixed64 area = Fixed64.Zero;
            int count = _vertices.Count;

            for (int i = 0; i < count; i++)
            {
                Vector2Fixed current = _vertices[i];
                Vector2Fixed next = _vertices[(i + 1) % count];
                area += current.x * next.y - next.x * current.y;
            }

            return area / (Fixed64)2;
        }

        /// <summary>
        /// 면적 (절대값)
        /// </summary>
        public Fixed64 Area()
        {
            Fixed64 signed = SignedArea();
            return signed < Fixed64.Zero ? -signed : signed;
        }

        /// <summary>
        /// 반시계 방향(CCW)인지 확인
        /// </summary>
        public bool IsCounterClockwise()
        {
            return SignedArea() > Fixed64.Zero;
        }

        /// <summary>
        /// 시계 방향(CW)인지 확인
        /// </summary>
        public bool IsClockwise()
        {
            return SignedArea() < Fixed64.Zero;
        }

        /// <summary>
        /// 정점 순서 반전
        /// </summary>
        public void Reverse()
        {
            _vertices.Reverse();
        }

        /// <summary>
        /// 반시계 방향으로 정규화
        /// </summary>
        public void EnsureCounterClockwise()
        {
            if (IsClockwise())
                Reverse();
        }

        /// <summary>
        /// 시계 방향으로 정규화
        /// </summary>
        public void EnsureClockwise()
        {
            if (IsCounterClockwise())
                Reverse();
        }

        /// <summary>
        /// 점이 폴리곤 내부에 있는지 확인 (Ray Casting Algorithm)
        /// </summary>
        public bool ContainsPoint(Vector2Fixed point)
        {
            if (_vertices.Count < 3)
                return false;

            int count = _vertices.Count;
            bool inside = false;

            for (int i = 0, j = count - 1; i < count; j = i++)
            {
                Vector2Fixed vi = _vertices[i];
                Vector2Fixed vj = _vertices[j];

                // Ray casting: 점에서 오른쪽으로 반직선을 쏘아 교차 횟수 계산
                if (((vi.y > point.y) != (vj.y > point.y)) &&
                    (point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x))
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        /// <summary>
        /// 점이 폴리곤 에지 위에 있는지 확인
        /// </summary>
        public bool IsPointOnEdge(Vector2Fixed point, Fixed64 tolerance)
        {
            int count = _vertices.Count;
            Fixed64 toleranceSqr = tolerance * tolerance;

            for (int i = 0; i < count; i++)
            {
                Vector2Fixed a = _vertices[i];
                Vector2Fixed b = _vertices[(i + 1) % count];

                if (PointToSegmentDistanceSqr(point, a, b) <= toleranceSqr)
                    return true;
            }

            return false;
        }

        /// <summary>
        /// 점에서 선분까지의 거리 제곱
        /// </summary>
        private static Fixed64 PointToSegmentDistanceSqr(Vector2Fixed p, Vector2Fixed a, Vector2Fixed b)
        {
            Vector2Fixed ab = b - a;
            Vector2Fixed ap = p - a;

            Fixed64 abLenSqr = ab.SqrMagnitude;
            if (abLenSqr == Fixed64.Zero)
                return ap.SqrMagnitude;

            Fixed64 t = Vector2Fixed.Dot(ap, ab) / abLenSqr;

            // Clamp t to [0, 1]
            if (t < Fixed64.Zero) t = Fixed64.Zero;
            if (t > Fixed64.One) t = Fixed64.One;

            Vector2Fixed closest = a + ab * t;
            return Vector2Fixed.SqrDistance(p, closest);
        }

        /// <summary>
        /// 선분이 폴리곤과 교차하는지 확인
        /// </summary>
        public bool IntersectsSegment(Vector2Fixed segA, Vector2Fixed segB)
        {
            int count = _vertices.Count;

            for (int i = 0; i < count; i++)
            {
                Vector2Fixed edgeA = _vertices[i];
                Vector2Fixed edgeB = _vertices[(i + 1) % count];

                if (SegmentsIntersect(segA, segB, edgeA, edgeB))
                    return true;
            }

            return false;
        }

        /// <summary>
        /// 두 선분이 교차하는지 확인
        /// </summary>
        private static bool SegmentsIntersect(Vector2Fixed a1, Vector2Fixed a2, Vector2Fixed b1, Vector2Fixed b2)
        {
            Fixed64 d1 = Vector2Fixed.Cross(b2 - b1, a1 - b1);
            Fixed64 d2 = Vector2Fixed.Cross(b2 - b1, a2 - b1);
            Fixed64 d3 = Vector2Fixed.Cross(a2 - a1, b1 - a1);
            Fixed64 d4 = Vector2Fixed.Cross(a2 - a1, b2 - a1);

            if (((d1 > Fixed64.Zero && d2 < Fixed64.Zero) || (d1 < Fixed64.Zero && d2 > Fixed64.Zero)) &&
                ((d3 > Fixed64.Zero && d4 < Fixed64.Zero) || (d3 < Fixed64.Zero && d4 > Fixed64.Zero)))
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// 볼록 폴리곤(Convex)인지 확인
        /// </summary>
        public bool IsConvex()
        {
            if (_vertices.Count < 3)
                return false;

            int count = _vertices.Count;
            bool? sign = null;

            for (int i = 0; i < count; i++)
            {
                Vector2Fixed a = _vertices[i];
                Vector2Fixed b = _vertices[(i + 1) % count];
                Vector2Fixed c = _vertices[(i + 2) % count];

                Fixed64 cross = Vector2Fixed.Cross(b - a, c - b);

                if (cross != Fixed64.Zero)
                {
                    bool currentSign = cross > Fixed64.Zero;
                    if (sign == null)
                        sign = currentSign;
                    else if (sign != currentSign)
                        return false;
                }
            }

            return true;
        }

        /// <summary>
        /// AABB (Axis-Aligned Bounding Box) 계산
        /// </summary>
        public Bounds2D GetBounds()
        {
            if (_vertices.Count == 0)
                return new Bounds2D(Vector2Fixed.Zero, Vector2Fixed.Zero);

            Fixed64 minX = _vertices[0].x;
            Fixed64 minY = _vertices[0].y;
            Fixed64 maxX = _vertices[0].x;
            Fixed64 maxY = _vertices[0].y;

            for (int i = 1; i < _vertices.Count; i++)
            {
                if (_vertices[i].x < minX) minX = _vertices[i].x;
                if (_vertices[i].y < minY) minY = _vertices[i].y;
                if (_vertices[i].x > maxX) maxX = _vertices[i].x;
                if (_vertices[i].y > maxY) maxY = _vertices[i].y;
            }

            return new Bounds2D(
                new Vector2Fixed(minX, minY),
                new Vector2Fixed(maxX, maxY)
            );
        }

        /// <summary>
        /// 중심점 계산
        /// </summary>
        public Vector2Fixed GetCentroid()
        {
            if (_vertices.Count == 0)
                return Vector2Fixed.Zero;

            Fixed64 sumX = Fixed64.Zero;
            Fixed64 sumY = Fixed64.Zero;

            foreach (var v in _vertices)
            {
                sumX += v.x;
                sumY += v.y;
            }

            Fixed64 count = (Fixed64)_vertices.Count;
            return new Vector2Fixed(sumX / count, sumY / count);
        }

        /// <summary>
        /// 복제
        /// </summary>
        public Polygon2D Clone()
        {
            return new Polygon2D(_vertices);
        }
    }

    /// <summary>
    /// 2D AABB (Axis-Aligned Bounding Box)
    /// </summary>
    [Serializable]
    public struct Bounds2D
    {
        public Vector2Fixed Min;
        public Vector2Fixed Max;

        public Vector2Fixed Center => new Vector2Fixed(
            (Min.x + Max.x) / (Fixed64)2,
            (Min.y + Max.y) / (Fixed64)2
        );

        public Vector2Fixed Size => new Vector2Fixed(
            Max.x - Min.x,
            Max.y - Min.y
        );

        public Bounds2D(Vector2Fixed min, Vector2Fixed max)
        {
            Min = min;
            Max = max;
        }

        public bool Contains(Vector2Fixed point)
        {
            return point.x >= Min.x && point.x <= Max.x &&
                   point.y >= Min.y && point.y <= Max.y;
        }

        public bool Intersects(Bounds2D other)
        {
            return Min.x <= other.Max.x && Max.x >= other.Min.x &&
                   Min.y <= other.Max.y && Max.y >= other.Min.y;
        }
    }
}
