using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

public class Code : MonoBehaviour
{
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
        
        // Class 인스턴스들의 메모리 주소 확인 (처음 10개만)
        UnityEngine.Debug.Log("=== Class 인스턴스의 실제 힙 메모리 주소 ===");
        for (int i = 0; i < 10; i++)
        {
            object obj = classArray[i];
            long address = Unsafe.As<object, IntPtr>(ref obj).ToInt64();
            UnityEngine.Debug.Log($"classArray[{i}] 실제 객체 주소: 0x{address:X16}");
        }
        
        // 실제 객체들 간의 주소 차이 확인
        object obj0 = classArray[0];
        object obj1 = classArray[1];
        long addr0 = Unsafe.As<object, IntPtr>(ref obj0).ToInt64();
        long addr1 = Unsafe.As<object, IntPtr>(ref obj1).ToInt64();
        long classDiff = Math.Abs(addr1 - addr0);
        UnityEngine.Debug.Log($"실제 객체 간 주소 차이: {classDiff} bytes (힙에 불규칙하게 배치)");
        
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
