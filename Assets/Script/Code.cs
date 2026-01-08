using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using FixedMathSharp;

public class Code : MonoBehaviour
{
    private const int COLLIDER_COUNT = 131;
    private const int TEST_ITERATIONS = 100; // 100프레임 시뮬레이션

    #region Physics Benchmark Structs

    // Fixed64/Vector2d 버전 (연산자 오버로딩)
    private struct ColliderDataFixed64
    {
        public Vector2d Position;
        public Fixed64 Radius;
        public bool IsTrigger;
        public int Layer;
    }

    // Native ulong 버전 (직접 연산, 양수만 존재)
    private struct ColliderDataNative
    {
        public ulong PosX;
        public ulong PosY;
        public ulong Radius;
        public bool IsTrigger;
        public int Layer;
    }

    #endregion

    [BurstCompile(CompileSynchronously = true, OptimizeFor = OptimizeFor.Performance)]
    struct SimpleAddJob : IJobParallelFor
    {
        [WriteOnly] public NativeArray<int> results;

        public void Execute(int index)
        {
            results[index] = 1 + 1;
        }
    }

    [BurstCompile(CompileSynchronously = true, OptimizeFor = OptimizeFor.Performance)]
    struct PhysicsCollisionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<ColliderDataNative> colliders;
        public int colliderCount;

        public void Execute(int i)
        {
            for (int j = i + 1; j < colliderCount; j++)
            {
                var a = colliders[i];
                var b = colliders[j];

                // 분기 예측 최적화: 조건 통합 및 순서 최적화
                // 1. 가장 흔한 케이스 먼저 (trigger 체크)
                int bothTriggers = (a.IsTrigger ? 1 : 0) & (b.IsTrigger ? 1 : 0);
                int sameLayers = a.Layer == b.Layer ? 1 : 0;
                int eitherTrigger = (a.IsTrigger ? 1 : 0) | (b.IsTrigger ? 1 : 0);

                // 단일 조건으로 통합 (브랜치 예측 향상)
                int shouldSkip = bothTriggers | (sameLayers & eitherTrigger);
                if (shouldSkip != 0) continue;

                // 거리 계산 (Native ulong 연산)
                ulong absDx = b.PosX > a.PosX ? b.PosX - a.PosX : a.PosX - b.PosX;
                ulong absDy = b.PosY > a.PosY ? b.PosY - a.PosY : a.PosY - b.PosY;
                ulong radiusSum = a.Radius + b.Radius;

                // AABB 사전 검사 (곱셈 없이 빠르게 거르기)
                if (absDx > radiusSum || absDy > radiusSum) continue;

                // 여기서부터 제곱 연산 (AABB 통과한 것만)
                ulong dx = absDx >> 16;
                ulong dy = absDy >> 16;
                ulong distSq = dx * dx + dy * dy;
                ulong radiusSumShifted = radiusSum >> 16;
                ulong radiusSumSq = radiusSumShifted * radiusSumShifted;

                // 충돌 검사만 (카운팅 없음)
                if (distSq <= radiusSumSq)
                {
                    // 충돌 발생 (처리 없음, 순수 연산만)
                }
            }
        }
    }

    [ContextMenu("Simple 1+1 * 17000")]
    private void SimpleAdd()
    {
        const int count = 17000; // Bound 최적화: 상수로 선언

        // ========== 일반 루프 (배열에 쓰기 - 공정한 비교) ==========
        int[] normalResults = new int[count];

        Stopwatch sw = Stopwatch.StartNew();
        int bound = count; // 루프 밖에서 평가되도록
        for (int i = 0; i < bound; i++)
        {
            normalResults[i] = 1 + 1;
        }
        sw.Stop();

        double microseconds = sw.ElapsedTicks * 1_000_000.0 / Stopwatch.Frequency;
        double nanoseconds = sw.ElapsedTicks * 1_000_000_000.0 / Stopwatch.Frequency;

        UnityEngine.Debug.Log($"=== Simple Add 1+1 * {count} (공정한 비교: 둘 다 배열에 쓰기) ===");
        UnityEngine.Debug.Log($"[일반 루프 - int[] 배열]");
        UnityEngine.Debug.Log($"  Ticks: {sw.ElapsedTicks}");
        UnityEngine.Debug.Log($"  Milliseconds: {sw.Elapsed.TotalMilliseconds:F6} ms");
        UnityEngine.Debug.Log($"  Microseconds: {microseconds:F3} μs");
        UnityEngine.Debug.Log($"  Nanoseconds: {nanoseconds:F1} ns");

        // ========== Burst Parallel Job ==========
        NativeArray<int> results = new NativeArray<int>(count, Allocator.TempJob);

        Stopwatch burstSw = Stopwatch.StartNew();
        var job = new SimpleAddJob { results = results };
        job.Schedule(count, 64).Complete();
        burstSw.Stop();

        double burstMicroseconds = burstSw.ElapsedTicks * 1_000_000.0 / Stopwatch.Frequency;
        double burstNanoseconds = burstSw.ElapsedTicks * 1_000_000_000.0 / Stopwatch.Frequency;

        UnityEngine.Debug.Log($"  [Burst Parallel Job - NativeArray]");
        UnityEngine.Debug.Log($"  Ticks: {burstSw.ElapsedTicks}");
        UnityEngine.Debug.Log($"  Milliseconds: {burstSw.Elapsed.TotalMilliseconds:F6} ms");
        UnityEngine.Debug.Log($"  Microseconds: {burstMicroseconds:F3} μs");
        UnityEngine.Debug.Log($"  Nanoseconds: {burstNanoseconds:F1} ns");

        double ratio = sw.Elapsed.TotalMilliseconds / burstSw.Elapsed.TotalMilliseconds;
        string winner = ratio < 1 ? "일반 루프 승리" : "Burst 승리";
        UnityEngine.Debug.Log($"\n비교: {Math.Abs(ratio):F2}x ({winner})");

        results.Dispose();
    }

    [ContextMenu("Physics Benchmark: Fixed64 vs Native ulong")]
    private void PhysicsBenchmark()
    {
        UnityEngine.Debug.Log($"=== Physics Collision Benchmark ===");
        UnityEngine.Debug.Log($"Colliders: {COLLIDER_COUNT}, Comparisons per frame: {COLLIDER_COUNT * COLLIDER_COUNT}");
        UnityEngine.Debug.Log($"Test iterations (frames): {TEST_ITERATIONS}");

        // 데이터 초기화
        var fixed64Data = new ColliderDataFixed64[COLLIDER_COUNT];
        var nativeData = new ColliderDataNative[COLLIDER_COUNT];

        System.Random rand = new System.Random(12345); // 결정론적 시드

        for (int i = 0; i < COLLIDER_COUNT; i++)
        {
            float x = (float)(rand.NextDouble() * 2048);
            float y = (float)(rand.NextDouble() * 2048);
            float r = (float)(rand.NextDouble() * 50 + 5); // 반경 5~55
            bool isTrigger = rand.Next(2) == 0;
            int layer = rand.Next(3); // 0, 1, 2

            // Fixed64 버전
            fixed64Data[i] = new ColliderDataFixed64
            {
                Position = new Vector2d((Fixed64)x, (Fixed64)y),
                Radius = (Fixed64)r,
                IsTrigger = isTrigger,
                Layer = layer
            };

            // Native 버전 (raw ulong 값 직접 저장 - 양수만)
            nativeData[i] = new ColliderDataNative
            {
                PosX = (ulong)((Fixed64)x).m_rawValue,
                PosY = (ulong)((Fixed64)y).m_rawValue,
                Radius = (ulong)((Fixed64)r).m_rawValue,
                IsTrigger = isTrigger,
                Layer = layer
            };
        }

        // ========== Fixed64 버전 테스트 ==========
        int fixed64Collisions = 0;
        Stopwatch fixed64Watch = Stopwatch.StartNew();

        for (int frame = 0; frame < TEST_ITERATIONS; frame++)
        {
            for (int i = 0; i < COLLIDER_COUNT; i++)
            {
                for (int j = i + 1; j < COLLIDER_COUNT; j++)
                {
                    ref var a = ref fixed64Data[i];
                    ref var b = ref fixed64Data[j];

                    // Skip trigger-trigger
                    if (a.IsTrigger && b.IsTrigger) continue;

                    // Same layer: only body-body
                    if (a.Layer == b.Layer)
                    {
                        if (a.IsTrigger || b.IsTrigger) continue;
                    }

                    // 거리 계산 (Fixed64 연산자 오버로딩)
                    Fixed64 dx = b.Position.x - a.Position.x;
                    Fixed64 dy = b.Position.y - a.Position.y;
                    Fixed64 distSq = dx * dx + dy * dy;
                    Fixed64 radiusSum = a.Radius + b.Radius;
                    Fixed64 radiusSumSq = radiusSum * radiusSum;

                    if (distSq <= radiusSumSq)
                    {
                        fixed64Collisions++;
                    }
                }
            }
        }

        fixed64Watch.Stop();

        // ========== Native Long 버전 테스트 ==========
        int nativeCollisions = 0;
        Stopwatch nativeWatch = Stopwatch.StartNew();

        for (int frame = 0; frame < TEST_ITERATIONS; frame++)
        {
            for (int i = 0; i < COLLIDER_COUNT; i++)
            {
                for (int j = i + 1; j < COLLIDER_COUNT; j++)
                {
                    ref var a = ref nativeData[i];
                    ref var b = ref nativeData[j];

                    // Skip trigger-trigger
                    if (a.IsTrigger && b.IsTrigger) continue;

                    // Same layer: only body-body
                    if (a.Layer == b.Layer)
                    {
                        if (a.IsTrigger || b.IsTrigger) continue;
                    }

                    // 거리 계산 (Native ulong 연산 - 양수만이라 절대값 차이 사용)
                    ulong absDx = b.PosX > a.PosX ? b.PosX - a.PosX : a.PosX - b.PosX;
                    ulong absDy = b.PosY > a.PosY ? b.PosY - a.PosY : a.PosY - b.PosY;
                    ulong dx = absDx >> 16;
                    ulong dy = absDy >> 16;
                    ulong distSq = dx * dx + dy * dy;
                    ulong radiusSum = (a.Radius + b.Radius) >> 16;
                    ulong radiusSumSq = radiusSum * radiusSum;

                    if (distSq <= radiusSumSq)
                    {
                        nativeCollisions++;
                    }
                }
            }
        }

        nativeWatch.Stop();

        // ========== 결과 출력 ==========
        UnityEngine.Debug.Log($"");
        UnityEngine.Debug.Log($"=== Results ===");
        UnityEngine.Debug.Log($"Fixed64 (operator overloading): {fixed64Watch.ElapsedMilliseconds} ms, Collisions: {fixed64Collisions}");
        UnityEngine.Debug.Log($"Native ulong (raw arithmetic):  {nativeWatch.ElapsedMilliseconds} ms, Collisions: {nativeCollisions}");
        UnityEngine.Debug.Log($"");
        UnityEngine.Debug.Log($"Speedup: {(double)fixed64Watch.ElapsedMilliseconds / nativeWatch.ElapsedMilliseconds:F2}x faster");
        UnityEngine.Debug.Log($"Time saved per frame: {(fixed64Watch.ElapsedMilliseconds - nativeWatch.ElapsedMilliseconds) / (double)TEST_ITERATIONS:F3} ms");
    }

    [ContextMenu("Physics Benchmark2: MainThread vs Burst Parallel")]
    private void PhysicsBenchmark2()
    {
        const int TEST_COUNT = 1000000;
        const int MIN_COLLIDERS = 50;
        const int MAX_COLLIDERS = 130;
        const double MEAN_COLLIDERS = 90.0;
        const double STD_DEV = 59.3; // 50과 130이 25% 지점에 오도록

        UnityEngine.Debug.Log($"=== Physics Benchmark2: 1000 iterations with varying collider counts ===");
        UnityEngine.Debug.Log($"Collider range: {MIN_COLLIDERS}-{MAX_COLLIDERS} (mean: {MEAN_COLLIDERS:F0}, std: {STD_DEV:F1})");

        System.Random rand = new System.Random(12345);
        
        // 최대 크기로 데이터 준비
        var nativeData = new ColliderDataNative[MAX_COLLIDERS];
        for (int i = 0; i < MAX_COLLIDERS; i++)
        {
            float x = (float)(rand.NextDouble() * 2048);
            float y = (float)(rand.NextDouble() * 2048);
            float r = (float)(rand.NextDouble() * 50 + 5);
            bool isTrigger = rand.Next(2) == 0;
            int layer = rand.Next(3);

            nativeData[i] = new ColliderDataNative
            {
                PosX = (ulong)((Fixed64)x).m_rawValue,
                PosY = (ulong)((Fixed64)y).m_rawValue,
                Radius = (ulong)((Fixed64)r).m_rawValue,
                IsTrigger = isTrigger,
                Layer = layer
            };
        }

        NativeArray<ColliderDataNative> nativeArray = new NativeArray<ColliderDataNative>(nativeData, Allocator.TempJob);
        
        double mainMinMs = double.MaxValue, mainMaxMs = 0, mainTotalMs = 0;
        double burstMinMs = double.MaxValue, burstMaxMs = 0, burstTotalMs = 0;

        for (int test = 0; test < TEST_COUNT; test++)
        {
            // 정규분포로 collider 수 결정 (Box-Muller 변환)
            double u1 = rand.NextDouble();
            double u2 = rand.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
            int activeColliders = (int)Math.Round(MEAN_COLLIDERS + STD_DEV * randStdNormal);
            activeColliders = Math.Max(MIN_COLLIDERS, Math.Min(MAX_COLLIDERS, activeColliders));

            // ========== 메인 스레드 버전 ==========
            Stopwatch mainWatch = Stopwatch.StartNew();

            for (int i = 0; i < activeColliders; i++)
            {
                for (int j = i + 1; j < activeColliders; j++)
                {
                    ref var a = ref nativeData[i];
                    ref var b = ref nativeData[j];

                    int bothTriggers = (a.IsTrigger ? 1 : 0) & (b.IsTrigger ? 1 : 0);
                    int sameLayers = a.Layer == b.Layer ? 1 : 0;
                    int eitherTrigger = (a.IsTrigger ? 1 : 0) | (b.IsTrigger ? 1 : 0);

                    int shouldSkip = bothTriggers | (sameLayers & eitherTrigger);
                    if (shouldSkip != 0) continue;

                    ulong absDx = b.PosX > a.PosX ? b.PosX - a.PosX : a.PosX - b.PosX;
                    ulong absDy = b.PosY > a.PosY ? b.PosY - a.PosY : a.PosY - b.PosY;
                    ulong radiusSum = a.Radius + b.Radius;

                    if (absDx > radiusSum || absDy > radiusSum) continue;

                    ulong dx = absDx >> 16;
                    ulong dy = absDy >> 16;
                    ulong distSq = dx * dx + dy * dy;
                    ulong radiusSumShifted = radiusSum >> 16;
                    ulong radiusSumSq = radiusSumShifted * radiusSumShifted;

                    if (distSq <= radiusSumSq)
                    {
                        // 충돌 발생
                    }
                }
            }

            mainWatch.Stop();
            double mainMs = mainWatch.Elapsed.TotalMilliseconds;
            mainMinMs = Math.Min(mainMinMs, mainMs);
            mainMaxMs = Math.Max(mainMaxMs, mainMs);
            mainTotalMs += mainMs;

            // ========== Burst Parallel Job 버전 ==========
            Stopwatch burstWatch = Stopwatch.StartNew();

            var job = new PhysicsCollisionJob
            {
                colliders = nativeArray,
                colliderCount = activeColliders
            };
            job.Schedule(activeColliders, 32).Complete();

            burstWatch.Stop();
            double burstMs = burstWatch.Elapsed.TotalMilliseconds;
            burstMinMs = Math.Min(burstMinMs, burstMs);
            burstMaxMs = Math.Max(burstMaxMs, burstMs);
            burstTotalMs += burstMs;
        }

        nativeArray.Dispose();

        double mainAvgMs = mainTotalMs / TEST_COUNT;
        double burstAvgMs = burstTotalMs / TEST_COUNT;

        // ========== 결과 출력 ==========
        UnityEngine.Debug.Log($"\n=== Results ({TEST_COUNT} iterations) ===");
        UnityEngine.Debug.Log($"MainThread  | Min: {mainMinMs:F3}ms, Max: {mainMaxMs:F3}ms, Avg: {mainAvgMs:F3}ms");
        UnityEngine.Debug.Log($"Burst Job  | Min: {burstMinMs:F3}ms, Max: {burstMaxMs:F3}ms, Avg: {burstAvgMs:F3}ms");
        UnityEngine.Debug.Log($"Avg Speedup: {mainAvgMs / burstAvgMs:F2}x");
    }

    unsafe public void foo()
    {
        int value = 77;
        IntPtr ptr = (IntPtr)(&value);

        int* dd = (int*)ptr.ToPointer();

        UnityEngine.Debug.Log(*dd + 1);
    }

    [ContextMenu("Test Massive cache")]
    private void Test()
    {
        int iterations = 9999999;

        // Class 배열 미리 생성 (방해 객체와 함께)
        CacheMissClass[] classArray = new CacheMissClass[iterations];
        DummyClass[] dummies = new DummyClass[iterations / 2];
        for (int i = 0; i < iterations; i++)
        {
            classArray[i] = new CacheMissClass(i, i * 2);
            if (i % 2 == 0 && i / 2 < dummies.Length)
            {
                // 중간에 DummyClass 생성해서 메모리 배치 방해
                dummies[i / 2] = new DummyClass(999);
            }
        }

        // Class 인스턴스들의 메모리 주소 확인 - Unity 6.3에서 Unsafe 미지원으로 비활성화
        // UnityEngine.Debug.Log("=== Class 인스턴스의 실제 힙 메모리 주소 ===");
        // for (int i = 0; i < 10; i++)
        // {
        //     object obj = classArray[i];
        //     long address = Unsafe.As<object, IntPtr>(ref obj).ToInt64();
        //     UnityEngine.Debug.Log($"classArray[{i}] 실제 객체 주소: 0x{address:X16}");
        // }

        // 실제 객체들 간의 주소 차이 확인 - Unity 6.3에서 Unsafe 미지원으로 비활성화
        // object obj0 = classArray[0];
        // object obj1 = classArray[1];
        // long addr0 = Unsafe.As<object, IntPtr>(ref obj0).ToInt64();
        // long addr1 = Unsafe.As<object, IntPtr>(ref obj1).ToInt64();
        // long classDiff = Math.Abs(addr1 - addr0);
        // UnityEngine.Debug.Log($"실제 객체 간 주소 차이: {classDiff} bytes (힙에 불규칙하게 배치)");

        // Class 연산만 측정
        Stopwatch classStopwatch = Stopwatch.StartNew();
        for (int i = 0; i < iterations; i++)
        {
            int result = classArray[i].Calculate();
        }
        classStopwatch.Stop();
        UnityEngine.Debug.Log($"Class 연산 소요 시간: {classStopwatch.ElapsedMilliseconds} ms");

        // Struct 배열 미리 생성
        CachePerfectStruct[] structArray = new CachePerfectStruct[iterations];
        for (int i = 0; i < iterations; i++)
        {
            structArray[i] = new CachePerfectStruct(i, i * 2);
        }

        // Struct 배열의 메모리 주소 확인 (처음 10개만)
        UnityEngine.Debug.Log("=== Struct 배열 메모리 주소 ===");
        unsafe
        {
            fixed (CachePerfectStruct* ptr = structArray)
            {
                for (int i = 0; i < 10; i++)
                {
                    long address = (long)(ptr + i);
                    UnityEngine.Debug.Log($"structArray[{i}] 주소: 0x{address:X16}");
                }

                long diff = (long)(ptr + 1) - (long)ptr;
                UnityEngine.Debug.Log($"연속된 Struct 간 주소 차이: {diff} bytes (완전히 연속적)");
            }
        }

        // Struct 연산만 측정 (일반 배열 인덱싱)
        Stopwatch structStopwatch = Stopwatch.StartNew();
        for (int i = 0; i < iterations; i++)
        {
            int result = structArray[i].Calculate();
        }
        structStopwatch.Stop();
        UnityEngine.Debug.Log($"Struct 일반 연산 소요 시간: {structStopwatch.ElapsedMilliseconds} ms");

        // Struct 연산 (Unsafe 포인터 - 초고속)
        Stopwatch unsafeStopwatch = Stopwatch.StartNew();
        unsafe
        {
            fixed (CachePerfectStruct* basePtr = structArray)
            {
                CachePerfectStruct* end = basePtr + iterations;
                for (CachePerfectStruct* ptr = basePtr; ptr < end; ptr++)
                {
                    int result = ptr->Calculate();
                }
            }
        }
        unsafeStopwatch.Stop();
        UnityEngine.Debug.Log($"Struct Unsafe 포인터 연산 소요 시간: {unsafeStopwatch.ElapsedMilliseconds} ms");

        // Struct 연산 (Burst Compile - 최고속)
        NativeArray<CachePerfectStruct> nativeData = new NativeArray<CachePerfectStruct>(structArray, Allocator.TempJob);
        NativeArray<int> nativeResults = new NativeArray<int>(iterations, Allocator.TempJob);

        Stopwatch burstStopwatch = Stopwatch.StartNew();
        var job = new CalculateJob
        {
            data = nativeData,
            results = nativeResults
        };
        job.Schedule(iterations, 64).Complete();
        burstStopwatch.Stop();
        UnityEngine.Debug.Log($"Struct Burst Compile 연산 소요 시간: {burstStopwatch.ElapsedMilliseconds} ms");

        nativeData.Dispose();
        nativeResults.Dispose();

        UnityEngine.Debug.Log($"=== 성능 비교 ===");
        UnityEngine.Debug.Log($"Class vs Struct(일반): {classStopwatch.ElapsedMilliseconds - structStopwatch.ElapsedMilliseconds} ms 차이");
        UnityEngine.Debug.Log($"Struct(일반) vs Struct(Unsafe): {structStopwatch.ElapsedMilliseconds - unsafeStopwatch.ElapsedMilliseconds} ms 차이");
        UnityEngine.Debug.Log($"Struct(Unsafe) vs Struct(Burst): {unsafeStopwatch.ElapsedMilliseconds - burstStopwatch.ElapsedMilliseconds} ms 차이");
        UnityEngine.Debug.Log($"Class vs Struct(Burst): {classStopwatch.ElapsedMilliseconds - burstStopwatch.ElapsedMilliseconds} ms 차이 (최대 속도)");
    }

    [BurstCompile(CompileSynchronously = true, OptimizeFor = OptimizeFor.Performance)]
    struct CalculateJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CachePerfectStruct> data;
        [WriteOnly] public NativeArray<int> results;

        public void Execute(int index)
        {
            results[index] = data[index].Calculate();
        }
    }

    private class CacheMissClass
    {
        private int x;
        private int y;
        public CacheMissClass(int a, int b)
        {
            x = a;
            y = b;
        }

        public int Calculate()
        {
            // 복잡한 연산 시뮬레이션
            int result = x * y;
            result += x / (y + 1);
            result -= (x % 10) * (y % 10);
            return result;
        }
    }

    private struct CachePerfectStruct
    {
        public int x;
        public int y;
        public CachePerfectStruct(int a, int b)
        {
            x = a;
            y = b;
        }

        public int Calculate()
        {
            // 복잡한 연산 시뮬레이션
            int result = x * y;
            result += x / (y + 1);
            result -= (x % 10) * (y % 10);
            return result;
        }
    }

    private class DummyClass
    {
        private long data1;
        private long data2;
        private long data3;
        public DummyClass(int val)
        {
            data1 = val;
            data2 = val * 2;
            data3 = val * 3;
        }
    }
}
