using System.Collections.Generic;
using FixedMathSharp;
using NavMesh2D.Geometry;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// Simple Stupid Funnel Algorithm
    /// 삼각형 경로를 통과하는 최단 경로를 계산
    ///
    /// 참고: https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html
    /// </summary>
    public class FunnelAlgorithm
    {
        /// <summary>
        /// 포탈 리스트에서 최단 경로 계산
        /// </summary>
        /// <param name="start">시작점</param>
        /// <param name="end">끝점</param>
        /// <param name="portals">포탈 리스트 (A*에서 생성)</param>
        /// <returns>최단 경로 웨이포인트 리스트</returns>
        public List<Vector2Fixed> StringPull(Vector2Fixed start, Vector2Fixed end, List<TriangleAStar.Portal> portals)
        {
            var path = new List<Vector2Fixed>();

            // 포탈이 없으면 직선 경로
            if (portals == null || portals.Count == 0)
            {
                path.Add(start);
                path.Add(end);
                return path;
            }

            // 포탈 리스트 복사 및 시작/끝 포탈 추가
            var portalList = new List<TriangleAStar.Portal>();

            // 시작점 포탈 (점)
            portalList.Add(new TriangleAStar.Portal(start, start));

            // 중간 포탈들
            portalList.AddRange(portals);

            // 끝점 포탈 (점)
            portalList.Add(new TriangleAStar.Portal(end, end));

            // Funnel 초기화
            Vector2Fixed apex = start;       // 깔때기 꼭지점
            Vector2Fixed left = start;       // 왼쪽 경계
            Vector2Fixed right = start;      // 오른쪽 경계
            int apexIndex = 0;
            int leftIndex = 0;
            int rightIndex = 0;

            path.Add(start);

            for (int i = 1; i < portalList.Count; i++)
            {
                Vector2Fixed portalLeft = portalList[i].Left;
                Vector2Fixed portalRight = portalList[i].Right;

                // 오른쪽 경계 업데이트 시도
                if (TriArea2(apex, right, portalRight) <= Fixed64.Zero)
                {
                    if (apex == right || TriArea2(apex, left, portalRight) > Fixed64.Zero)
                    {
                        // 오른쪽 좁히기
                        right = portalRight;
                        rightIndex = i;
                    }
                    else
                    {
                        // 오른쪽이 왼쪽을 넘어감 - 왼쪽을 새 apex로
                        // 중복 점 체크
                        if (path[path.Count - 1] != left)
                        {
                            path.Add(left);
                        }

                        // 새 apex에서 다시 시작
                        apex = left;
                        apexIndex = leftIndex;

                        // 포탈 인덱스 리셋
                        left = apex;
                        right = apex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        // 다음 포탈부터 다시 스캔
                        i = apexIndex;
                        continue;
                    }
                }

                // 왼쪽 경계 업데이트 시도
                if (TriArea2(apex, left, portalLeft) >= Fixed64.Zero)
                {
                    if (apex == left || TriArea2(apex, right, portalLeft) < Fixed64.Zero)
                    {
                        // 왼쪽 좁히기
                        left = portalLeft;
                        leftIndex = i;
                    }
                    else
                    {
                        // 왼쪽이 오른쪽을 넘어감 - 오른쪽을 새 apex로
                        // 중복 점 체크
                        if (path[path.Count - 1] != right)
                        {
                            path.Add(right);
                        }

                        // 새 apex에서 다시 시작
                        apex = right;
                        apexIndex = rightIndex;

                        // 포탈 인덱스 리셋
                        left = apex;
                        right = apex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        // 다음 포탈부터 다시 스캔
                        i = apexIndex;
                        continue;
                    }
                }
            }

            // 끝점 추가 (이미 추가되지 않았다면)
            if (path.Count == 0 || path[path.Count - 1] != end)
            {
                path.Add(end);
            }

            return path;
        }

        /// <summary>
        /// 삼각형 부호 있는 면적의 2배 계산 (Mikko Mononen 공식)
        /// 이 공식은 StringPull의 비교 연산자와 맞춰져 있음
        /// Y-up 좌표계에서: 음수 = 왼쪽, 양수 = 오른쪽
        /// </summary>
        private Fixed64 TriArea2(Vector2Fixed a, Vector2Fixed b, Vector2Fixed c)
        {
            Fixed64 ax = b.x - a.x;
            Fixed64 ay = b.y - a.y;
            Fixed64 bx = c.x - a.x;
            Fixed64 by = c.y - a.y;
            return bx * ay - ax * by;
        }

        /// <summary>
        /// 포탈 방향 정규화 (왼쪽/오른쪽이 진행 방향 기준으로 올바른지 확인)
        /// </summary>
        /// <param name="portals">포탈 리스트</param>
        /// <param name="start">시작점</param>
        /// <returns>정규화된 포탈 리스트</returns>
        public List<TriangleAStar.Portal> NormalizePortals(List<TriangleAStar.Portal> portals, Vector2Fixed start)
        {
            if (portals == null || portals.Count == 0)
                return portals;

            var normalized = new List<TriangleAStar.Portal>(portals.Count);
            Vector2Fixed prevCenter = start;

            for (int i = 0; i < portals.Count; i++)
            {
                var portal = portals[i];
                Vector2Fixed center = new Vector2Fixed(
                    (portal.Left.x + portal.Right.x) / (Fixed64)2,
                    (portal.Left.y + portal.Right.y) / (Fixed64)2
                );

                // 진행 방향
                Vector2Fixed dir = center - prevCenter;

                // 포탈 벡터 (Left → Right)
                Vector2Fixed portalVec = portal.Right - portal.Left;

                // 외적으로 방향 확인 (dir × portalVec)
                // dir이 진행 방향일 때, Left→Right 벡터가 왼쪽에서 오른쪽으로 가야 함
                // 즉, portalVec은 dir의 오른쪽(시계방향)에 있어야 함 → cross < 0 이 정상
                Fixed64 cross = dir.x * portalVec.y - dir.y * portalVec.x;

                if (cross > Fixed64.Zero)
                {
                    // portalVec이 왼쪽에 있음 (반대 방향) → 스왑
                    normalized.Add(new TriangleAStar.Portal(portal.Right, portal.Left));
                }
                else
                {
                    normalized.Add(portal);
                }

                prevCenter = center;
            }

            return normalized;
        }
    }
}
