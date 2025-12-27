using System.Collections.Generic;
using System.Runtime.CompilerServices;
using FixedMathSharp;
using NavMesh2D.Geometry;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// Simple Stupid Funnel Algorithm (최적화 버전)
    /// 삼각형 경로를 통과하는 최단 경로를 계산
    /// - List 재사용으로 GC 최소화
    /// - Raw long 연산으로 Fixed64 오버헤드 제거
    /// - NormalizePortals 인라인화
    /// </summary>
    public class FunnelAlgorithm
    {
        // 재사용 버퍼
        private readonly List<Vector2Fixed> _pathBuffer = new List<Vector2Fixed>(32);
        private readonly List<long> _portalLeftX = new List<long>(32);
        private readonly List<long> _portalLeftY = new List<long>(32);
        private readonly List<long> _portalRightX = new List<long>(32);
        private readonly List<long> _portalRightY = new List<long>(32);

        /// <summary>
        /// 포탈 리스트에서 최단 경로 계산 (NormalizePortals 인라인)
        /// </summary>
        public List<Vector2Fixed> StringPull(Vector2Fixed start, Vector2Fixed end, List<TriangleAStar.Portal> portals)
        {
            _pathBuffer.Clear();

            // 포탈이 없으면 직선 경로
            if (portals == null || portals.Count == 0)
            {
                _pathBuffer.Add(start);
                _pathBuffer.Add(end);
                return _pathBuffer;
            }

            // 포탈을 raw long 배열로 변환 + 정규화 인라인
            _portalLeftX.Clear();
            _portalLeftY.Clear();
            _portalRightX.Clear();
            _portalRightY.Clear();

            long startX = start.x.m_rawValue;
            long startY = start.y.m_rawValue;
            long endX = end.x.m_rawValue;
            long endY = end.y.m_rawValue;

            // 시작점 포탈
            _portalLeftX.Add(startX);
            _portalLeftY.Add(startY);
            _portalRightX.Add(startX);
            _portalRightY.Add(startY);

            // 중간 포탈들 (정규화 인라인)
            long prevCenterX = startX;
            long prevCenterY = startY;

            for (int i = 0; i < portals.Count; i++)
            {
                long lx = portals[i].Left.x.m_rawValue;
                long ly = portals[i].Left.y.m_rawValue;
                long rx = portals[i].Right.x.m_rawValue;
                long ry = portals[i].Right.y.m_rawValue;

                // 포탈 중심
                long centerX = (lx + rx) >> 1;
                long centerY = (ly + ry) >> 1;

                // 진행 방향
                long dirX = centerX - prevCenterX;
                long dirY = centerY - prevCenterY;

                // 포탈 벡터 (Left → Right)
                long portalVecX = rx - lx;
                long portalVecY = ry - ly;

                // 외적 (dir × portalVec) - 방향 확인
                // cross > 0 이면 스왑 필요
                long cross = MulRaw(dirX, portalVecY) - MulRaw(dirY, portalVecX);

                if (cross > 0)
                {
                    // 스왑
                    _portalLeftX.Add(rx);
                    _portalLeftY.Add(ry);
                    _portalRightX.Add(lx);
                    _portalRightY.Add(ly);
                }
                else
                {
                    _portalLeftX.Add(lx);
                    _portalLeftY.Add(ly);
                    _portalRightX.Add(rx);
                    _portalRightY.Add(ry);
                }

                prevCenterX = centerX;
                prevCenterY = centerY;
            }

            // 끝점 포탈
            _portalLeftX.Add(endX);
            _portalLeftY.Add(endY);
            _portalRightX.Add(endX);
            _portalRightY.Add(endY);

            int portalCount = _portalLeftX.Count;

            // Funnel 초기화
            long apexX = startX, apexY = startY;
            long leftX = startX, leftY = startY;
            long rightX = startX, rightY = startY;
            int apexIndex = 0;
            int leftIndex = 0;
            int rightIndex = 0;

            _pathBuffer.Add(start);

            for (int i = 1; i < portalCount; i++)
            {
                long portalLeftX = _portalLeftX[i];
                long portalLeftY = _portalLeftY[i];
                long portalRightX = _portalRightX[i];
                long portalRightY = _portalRightY[i];

                // 오른쪽 경계 업데이트 시도
                if (TriArea2Raw(apexX, apexY, rightX, rightY, portalRightX, portalRightY) <= 0)
                {
                    if ((apexX == rightX && apexY == rightY) ||
                        TriArea2Raw(apexX, apexY, leftX, leftY, portalRightX, portalRightY) > 0)
                    {
                        // 오른쪽 좁히기
                        rightX = portalRightX;
                        rightY = portalRightY;
                        rightIndex = i;
                    }
                    else
                    {
                        // 왼쪽을 새 apex로
                        var leftPoint = new Vector2Fixed(
                            Fixed64.FromRaw(leftX),
                            Fixed64.FromRaw(leftY));

                        if (_pathBuffer.Count == 0 || _pathBuffer[_pathBuffer.Count - 1] != leftPoint)
                        {
                            _pathBuffer.Add(leftPoint);
                        }

                        apexX = leftX;
                        apexY = leftY;
                        apexIndex = leftIndex;

                        leftX = apexX;
                        leftY = apexY;
                        rightX = apexX;
                        rightY = apexY;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        i = apexIndex;
                        continue;
                    }
                }

                // 왼쪽 경계 업데이트 시도
                if (TriArea2Raw(apexX, apexY, leftX, leftY, portalLeftX, portalLeftY) >= 0)
                {
                    if ((apexX == leftX && apexY == leftY) ||
                        TriArea2Raw(apexX, apexY, rightX, rightY, portalLeftX, portalLeftY) < 0)
                    {
                        // 왼쪽 좁히기
                        leftX = portalLeftX;
                        leftY = portalLeftY;
                        leftIndex = i;
                    }
                    else
                    {
                        // 오른쪽을 새 apex로
                        var rightPoint = new Vector2Fixed(
                            Fixed64.FromRaw(rightX),
                            Fixed64.FromRaw(rightY));

                        if (_pathBuffer.Count == 0 || _pathBuffer[_pathBuffer.Count - 1] != rightPoint)
                        {
                            _pathBuffer.Add(rightPoint);
                        }

                        apexX = rightX;
                        apexY = rightY;
                        apexIndex = rightIndex;

                        leftX = apexX;
                        leftY = apexY;
                        rightX = apexX;
                        rightY = apexY;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        i = apexIndex;
                        continue;
                    }
                }
            }

            // 끝점 추가
            if (_pathBuffer.Count == 0 || _pathBuffer[_pathBuffer.Count - 1] != end)
            {
                _pathBuffer.Add(end);
            }

            return _pathBuffer;
        }

        /// <summary>
        /// 삼각형 부호 있는 면적의 2배 (raw long)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long TriArea2Raw(long ax, long ay, long bx, long by, long cx, long cy)
        {
            long abx = bx - ax;
            long aby = by - ay;
            long acx = cx - ax;
            long acy = cy - ay;
            return MulRaw(acx, aby) - MulRaw(abx, acy);
        }

        /// <summary>
        /// Fixed64 곱셈 (raw long)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long MulRaw(long a, long b)
        {
            bool negA = a < 0;
            bool negB = b < 0;
            if (negA) a = -a;
            if (negB) b = -b;

            ulong aLo = (ulong)a & 0xFFFFFFFF;
            ulong aHi = (ulong)a >> 32;
            ulong bLo = (ulong)b & 0xFFFFFFFF;
            ulong bHi = (ulong)b >> 32;

            ulong loLo = aLo * bLo;
            ulong loHi = aLo * bHi;
            ulong hiLo = aHi * bLo;
            ulong hiHi = aHi * bHi;

            ulong mid = loHi + hiLo + (loLo >> 32);
            ulong result = hiHi + (mid >> 32);
            result = (result << 32) | (mid & 0xFFFFFFFF);

            if ((loLo & 0x80000000) != 0)
                result++;

            long signedResult = (long)result;
            return (negA != negB) ? -signedResult : signedResult;
        }

        /// <summary>
        /// 포탈 방향 정규화 (하위 호환용 - 이제 StringPull에 인라인됨)
        /// </summary>
        public List<TriangleAStar.Portal> NormalizePortals(List<TriangleAStar.Portal> portals, Vector2Fixed start)
        {
            // StringPull에서 인라인으로 처리하므로 그냥 반환
            return portals;
        }
    }
}
