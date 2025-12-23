using System;
using FixedMathSharp;

namespace NavMesh2D.Geometry
{
    /// <summary>
    /// Fixed64 기반 2D 벡터 (결정론적 연산용)
    /// FixedMathSharp의 Vector2d를 래핑하여 추가 기능 제공
    /// </summary>
    [Serializable]
    public struct Vector2Fixed : IEquatable<Vector2Fixed>
    {
        public Fixed64 x;
        public Fixed64 y;

        public static readonly Vector2Fixed Zero = new Vector2Fixed(Fixed64.Zero, Fixed64.Zero);
        public static readonly Vector2Fixed One = new Vector2Fixed(Fixed64.One, Fixed64.One);

        public Vector2Fixed(Fixed64 x, Fixed64 y)
        {
            this.x = x;
            this.y = y;
        }

        public Vector2Fixed(float x, float y)
        {
            this.x = (Fixed64)x;
            this.y = (Fixed64)y;
        }

        public Vector2Fixed(double x, double y)
        {
            this.x = (Fixed64)x;
            this.y = (Fixed64)y;
        }

        public Vector2Fixed(Vector2d v)
        {
            this.x = v.x;
            this.y = v.y;
        }

        /// <summary>
        /// 벡터 크기의 제곱 (sqrt 없이 비교용)
        /// </summary>
        public Fixed64 SqrMagnitude => x * x + y * y;

        /// <summary>
        /// 벡터 크기
        /// </summary>
        public Fixed64 Magnitude => FixedMath.Sqrt(SqrMagnitude);

        /// <summary>
        /// 정규화된 벡터
        /// </summary>
        public Vector2Fixed Normalized
        {
            get
            {
                Fixed64 mag = Magnitude;
                if (mag == Fixed64.Zero)
                    return Zero;
                return new Vector2Fixed(x / mag, y / mag);
            }
        }

        /// <summary>
        /// 두 벡터 사이의 거리 제곱
        /// </summary>
        public static Fixed64 SqrDistance(Vector2Fixed a, Vector2Fixed b)
        {
            Fixed64 dx = b.x - a.x;
            Fixed64 dy = b.y - a.y;
            return dx * dx + dy * dy;
        }

        /// <summary>
        /// 두 벡터 사이의 거리
        /// </summary>
        public static Fixed64 Distance(Vector2Fixed a, Vector2Fixed b)
        {
            return FixedMath.Sqrt(SqrDistance(a, b));
        }

        /// <summary>
        /// 내적
        /// </summary>
        public static Fixed64 Dot(Vector2Fixed a, Vector2Fixed b)
        {
            return a.x * b.x + a.y * b.y;
        }

        /// <summary>
        /// 2D 외적 (z 성분, 스칼라 값)
        /// 양수: b가 a의 반시계 방향
        /// 음수: b가 a의 시계 방향
        /// 0: 평행
        /// </summary>
        public static Fixed64 Cross(Vector2Fixed a, Vector2Fixed b)
        {
            return a.x * b.y - a.y * b.x;
        }

        /// <summary>
        /// 세 점의 방향 (CCW 테스트)
        /// 양수: 반시계 방향 (CCW)
        /// 음수: 시계 방향 (CW)
        /// 0: 일직선
        /// </summary>
        public static Fixed64 Cross(Vector2Fixed origin, Vector2Fixed a, Vector2Fixed b)
        {
            return Cross(a - origin, b - origin);
        }

        /// <summary>
        /// Vector2d로 변환
        /// </summary>
        public Vector2d ToVector2d()
        {
            return new Vector2d(x, y);
        }

        /// <summary>
        /// UnityEngine.Vector2로 변환
        /// </summary>
        public UnityEngine.Vector2 ToVector2()
        {
            return new UnityEngine.Vector2((float)x, (float)y);
        }

        /// <summary>
        /// UnityEngine.Vector3로 변환 (z = 0)
        /// </summary>
        public UnityEngine.Vector3 ToVector3(float z = 0f)
        {
            return new UnityEngine.Vector3((float)x, (float)y, z);
        }

        public static Vector2Fixed operator +(Vector2Fixed a, Vector2Fixed b)
        {
            return new Vector2Fixed(a.x + b.x, a.y + b.y);
        }

        public static Vector2Fixed operator -(Vector2Fixed a, Vector2Fixed b)
        {
            return new Vector2Fixed(a.x - b.x, a.y - b.y);
        }

        public static Vector2Fixed operator *(Vector2Fixed a, Fixed64 scalar)
        {
            return new Vector2Fixed(a.x * scalar, a.y * scalar);
        }

        public static Vector2Fixed operator *(Fixed64 scalar, Vector2Fixed a)
        {
            return new Vector2Fixed(a.x * scalar, a.y * scalar);
        }

        public static Vector2Fixed operator /(Vector2Fixed a, Fixed64 scalar)
        {
            return new Vector2Fixed(a.x / scalar, a.y / scalar);
        }

        public static Vector2Fixed operator -(Vector2Fixed a)
        {
            return new Vector2Fixed(-a.x, -a.y);
        }

        public static bool operator ==(Vector2Fixed a, Vector2Fixed b)
        {
            return a.x == b.x && a.y == b.y;
        }

        public static bool operator !=(Vector2Fixed a, Vector2Fixed b)
        {
            return a.x != b.x || a.y != b.y;
        }

        public bool Equals(Vector2Fixed other)
        {
            return x == other.x && y == other.y;
        }

        public override bool Equals(object obj)
        {
            return obj is Vector2Fixed other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(x, y);
        }

        public override string ToString()
        {
            return $"({(float)x:F2}, {(float)y:F2})";
        }
    }
}
