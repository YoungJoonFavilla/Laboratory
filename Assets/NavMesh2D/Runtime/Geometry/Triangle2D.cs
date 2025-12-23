using System;
using FixedMathSharp;

namespace NavMesh2D.Geometry
{
    /// <summary>
    /// 2D 삼각형
    /// 세 정점으로 정의되며, 외접원 계산 및 점 포함 검사 지원
    /// </summary>
    [Serializable]
    public struct Triangle2D : IEquatable<Triangle2D>
    {
        public Vector2Fixed V0;
        public Vector2Fixed V1;
        public Vector2Fixed V2;

        public Triangle2D(Vector2Fixed v0, Vector2Fixed v1, Vector2Fixed v2)
        {
            V0 = v0;
            V1 = v1;
            V2 = v2;
        }

        /// <summary>
        /// 정점 인덱스로 접근
        /// </summary>
        public Vector2Fixed this[int index]
        {
            get
            {
                return index switch
                {
                    0 => V0,
                    1 => V1,
                    2 => V2,
                    _ => throw new IndexOutOfRangeException()
                };
            }
            set
            {
                switch (index)
                {
                    case 0: V0 = value; break;
                    case 1: V1 = value; break;
                    case 2: V2 = value; break;
                    default: throw new IndexOutOfRangeException();
                }
            }
        }

        /// <summary>
        /// 에지 가져오기 (0: V0-V1, 1: V1-V2, 2: V2-V0)
        /// </summary>
        public (Vector2Fixed A, Vector2Fixed B) GetEdge(int index)
        {
            return index switch
            {
                0 => (V0, V1),
                1 => (V1, V2),
                2 => (V2, V0),
                _ => throw new IndexOutOfRangeException()
            };
        }

        /// <summary>
        /// 부호 있는 면적 (Shoelace formula)
        /// 양수: 반시계 방향 (CCW)
        /// 음수: 시계 방향 (CW)
        /// </summary>
        public Fixed64 SignedArea()
        {
            return (V1.x - V0.x) * (V2.y - V0.y) - (V2.x - V0.x) * (V1.y - V0.y);
        }

        /// <summary>
        /// 면적 (절대값)
        /// </summary>
        public Fixed64 Area()
        {
            Fixed64 signed = SignedArea();
            return (signed < Fixed64.Zero ? -signed : signed) / (Fixed64)2;
        }

        /// <summary>
        /// 반시계 방향(CCW)인지 확인
        /// </summary>
        public bool IsCounterClockwise()
        {
            return SignedArea() > Fixed64.Zero;
        }

        /// <summary>
        /// 중심점 (Centroid)
        /// </summary>
        public Vector2Fixed Centroid => new Vector2Fixed(
            (V0.x + V1.x + V2.x) / (Fixed64)3,
            (V0.y + V1.y + V2.y) / (Fixed64)3
        );

        /// <summary>
        /// 점이 삼각형 내부에 있는지 확인 (Barycentric 좌표)
        /// </summary>
        public bool ContainsPoint(Vector2Fixed p)
        {
            Fixed64 d1 = Sign(p, V0, V1);
            Fixed64 d2 = Sign(p, V1, V2);
            Fixed64 d3 = Sign(p, V2, V0);

            bool hasNeg = (d1 < Fixed64.Zero) || (d2 < Fixed64.Zero) || (d3 < Fixed64.Zero);
            bool hasPos = (d1 > Fixed64.Zero) || (d2 > Fixed64.Zero) || (d3 > Fixed64.Zero);

            return !(hasNeg && hasPos);
        }

        private static Fixed64 Sign(Vector2Fixed p1, Vector2Fixed p2, Vector2Fixed p3)
        {
            return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
        }

        /// <summary>
        /// 점이 삼각형 내부에 있는지 확인 (에지 제외)
        /// </summary>
        public bool ContainsPointStrict(Vector2Fixed p)
        {
            Fixed64 d1 = Sign(p, V0, V1);
            Fixed64 d2 = Sign(p, V1, V2);
            Fixed64 d3 = Sign(p, V2, V0);

            bool allPos = (d1 > Fixed64.Zero) && (d2 > Fixed64.Zero) && (d3 > Fixed64.Zero);
            bool allNeg = (d1 < Fixed64.Zero) && (d2 < Fixed64.Zero) && (d3 < Fixed64.Zero);

            return allPos || allNeg;
        }

        /// <summary>
        /// 외접원의 중심점 (Circumcenter)
        /// </summary>
        public Vector2Fixed Circumcenter()
        {
            Fixed64 ax = V0.x, ay = V0.y;
            Fixed64 bx = V1.x, by = V1.y;
            Fixed64 cx = V2.x, cy = V2.y;

            Fixed64 d = (Fixed64)2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

            if (d == Fixed64.Zero)
            {
                // 퇴화 삼각형 (세 점이 일직선)
                return Centroid;
            }

            Fixed64 ax2 = ax * ax;
            Fixed64 ay2 = ay * ay;
            Fixed64 bx2 = bx * bx;
            Fixed64 by2 = by * by;
            Fixed64 cx2 = cx * cx;
            Fixed64 cy2 = cy * cy;

            Fixed64 ux = ((ax2 + ay2) * (by - cy) + (bx2 + by2) * (cy - ay) + (cx2 + cy2) * (ay - by)) / d;
            Fixed64 uy = ((ax2 + ay2) * (cx - bx) + (bx2 + by2) * (ax - cx) + (cx2 + cy2) * (bx - ax)) / d;

            return new Vector2Fixed(ux, uy);
        }

        /// <summary>
        /// 외접원의 반지름 제곱 (Circumradius Squared)
        /// </summary>
        public Fixed64 CircumradiusSqr()
        {
            Vector2Fixed center = Circumcenter();
            return Vector2Fixed.SqrDistance(center, V0);
        }

        /// <summary>
        /// 점이 외접원 내부에 있는지 확인 (들로네 조건)
        /// true: 외접원 내부 (들로네 조건 위반)
        /// false: 외접원 외부 또는 경계
        /// </summary>
        public bool CircumcircleContains(Vector2Fixed p)
        {
            // 외접원 테스트를 위한 행렬식 계산
            // | ax-px  ay-py  (ax-px)^2+(ay-py)^2 |
            // | bx-px  by-py  (bx-px)^2+(by-py)^2 | > 0 이면 내부
            // | cx-px  cy-py  (cx-px)^2+(cy-py)^2 |

            Fixed64 ax = V0.x - p.x;
            Fixed64 ay = V0.y - p.y;
            Fixed64 bx = V1.x - p.x;
            Fixed64 by = V1.y - p.y;
            Fixed64 cx = V2.x - p.x;
            Fixed64 cy = V2.y - p.y;

            Fixed64 ab = ax * ax + ay * ay;
            Fixed64 cd = bx * bx + by * by;
            Fixed64 ef = cx * cx + cy * cy;

            Fixed64 det = ax * (by * ef - cd * cy) -
                          ay * (bx * ef - cd * cx) +
                          ab * (bx * cy - by * cx);

            // CCW 삼각형이면 det > 0이 내부
            // CW 삼각형이면 det < 0이 내부
            if (IsCounterClockwise())
                return det > Fixed64.Zero;
            else
                return det < Fixed64.Zero;
        }

        /// <summary>
        /// 특정 정점을 포함하는지 확인
        /// </summary>
        public bool HasVertex(Vector2Fixed v)
        {
            return V0 == v || V1 == v || V2 == v;
        }

        /// <summary>
        /// 특정 에지를 공유하는지 확인
        /// </summary>
        public bool SharesEdge(Triangle2D other)
        {
            int sharedVertices = 0;

            if (other.HasVertex(V0)) sharedVertices++;
            if (other.HasVertex(V1)) sharedVertices++;
            if (other.HasVertex(V2)) sharedVertices++;

            return sharedVertices >= 2;
        }

        /// <summary>
        /// 공유하는 에지 가져오기 (없으면 null)
        /// </summary>
        public (Vector2Fixed A, Vector2Fixed B)? GetSharedEdge(Triangle2D other)
        {
            for (int i = 0; i < 3; i++)
            {
                var edge = GetEdge(i);
                if (other.HasVertex(edge.A) && other.HasVertex(edge.B))
                    return edge;
            }
            return null;
        }

        /// <summary>
        /// 에지 반대편 정점 가져오기
        /// </summary>
        public Vector2Fixed GetOppositeVertex(Vector2Fixed a, Vector2Fixed b)
        {
            if (V0 != a && V0 != b) return V0;
            if (V1 != a && V1 != b) return V1;
            return V2;
        }

        /// <summary>
        /// AABB 계산
        /// </summary>
        public Bounds2D GetBounds()
        {
            Fixed64 minX = V0.x, maxX = V0.x;
            Fixed64 minY = V0.y, maxY = V0.y;

            if (V1.x < minX) minX = V1.x; if (V1.x > maxX) maxX = V1.x;
            if (V1.y < minY) minY = V1.y; if (V1.y > maxY) maxY = V1.y;
            if (V2.x < minX) minX = V2.x; if (V2.x > maxX) maxX = V2.x;
            if (V2.y < minY) minY = V2.y; if (V2.y > maxY) maxY = V2.y;

            return new Bounds2D(
                new Vector2Fixed(minX, minY),
                new Vector2Fixed(maxX, maxY)
            );
        }

        public bool Equals(Triangle2D other)
        {
            return V0 == other.V0 && V1 == other.V1 && V2 == other.V2;
        }

        public override bool Equals(object obj)
        {
            return obj is Triangle2D other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(V0, V1, V2);
        }

        public static bool operator ==(Triangle2D a, Triangle2D b)
        {
            return a.Equals(b);
        }

        public static bool operator !=(Triangle2D a, Triangle2D b)
        {
            return !a.Equals(b);
        }

        public override string ToString()
        {
            return $"Triangle({V0}, {V1}, {V2})";
        }
    }
}
