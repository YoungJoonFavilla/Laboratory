using System.Runtime.CompilerServices;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// A*용 Indexed Min-Heap (struct 기반 최적화)
    /// - FScore 캐싱으로 비교 시 덧셈 제거
    /// - struct로 힙 할당 제거
    /// - AggressiveInlining으로 호출 오버헤드 제거
    /// </summary>
    public struct IndexedMinHeap
    {
        private struct HeapNode
        {
            public int TriangleIndex;
            public long FScore;  // G+H 미리 계산해서 저장
        }

        private HeapNode[] _heap;
        private int[] _heapPosition;
        private int[] _positionGeneration;
        private int _heapCount;
        private int _generation;
        private int _maxCapacity;
        private bool _initialized;

        public int Count
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => _heapCount;
        }

        public IndexedMinHeap(int maxTriangleCount)
        {
            _maxCapacity = maxTriangleCount;
            _heap = new HeapNode[maxTriangleCount];
            _heapPosition = new int[maxTriangleCount];
            _positionGeneration = new int[maxTriangleCount];
            _heapCount = 0;
            _generation = 0;
            _initialized = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            _heapCount = 0;
            _generation++;

            if (_generation == int.MaxValue)
            {
                _generation = 1;
                System.Array.Clear(_positionGeneration, 0, _positionGeneration.Length);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(int triangleIndex)
        {
            return _positionGeneration[triangleIndex] == _generation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Insert(int triangleIndex, long g, long h)
        {
            int index = _heapCount;
            _heap[index].TriangleIndex = triangleIndex;
            _heap[index].FScore = g + h;  // 미리 계산
            _heapPosition[triangleIndex] = index;
            _positionGeneration[triangleIndex] = _generation;
            _heapCount++;
            SiftUp(index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ExtractMin(out int triangleIndex)
        {
            triangleIndex = _heap[0].TriangleIndex;

            _heapCount--;
            if (_heapCount > 0)
            {
                _heap[0] = _heap[_heapCount];
                _heapPosition[_heap[0].TriangleIndex] = 0;
                SiftDown(0);
            }

            _positionGeneration[triangleIndex] = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdatePriority(int triangleIndex, long newG, long newH)
        {
            if (_positionGeneration[triangleIndex] != _generation)
                return;

            int index = _heapPosition[triangleIndex];
            long oldF = _heap[index].FScore;
            long newF = newG + newH;
            _heap[index].FScore = newF;

            if (newF < oldF)
            {
                SiftUp(index);
            }
            else if (newF > oldF)
            {
                SiftDown(index);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SiftUp(int index)
        {
            HeapNode node = _heap[index];
            while (index > 0)
            {
                int parent = (index - 1) >> 1;
                if (node.FScore >= _heap[parent].FScore)
                    break;

                _heap[index] = _heap[parent];
                _heapPosition[_heap[index].TriangleIndex] = index;
                index = parent;
            }
            _heap[index] = node;
            _heapPosition[node.TriangleIndex] = index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SiftDown(int index)
        {
            HeapNode node = _heap[index];

            while (true)
            {
                int smallest = index;
                long smallestF = node.FScore;
                int left = (index << 1) + 1;
                int right = (index << 1) + 2;

                if (left < _heapCount && _heap[left].FScore < smallestF)
                {
                    smallest = left;
                    smallestF = _heap[left].FScore;
                }

                if (right < _heapCount && _heap[right].FScore < smallestF)
                {
                    smallest = right;
                }

                if (smallest == index)
                    break;

                _heap[index] = _heap[smallest];
                _heapPosition[_heap[index].TriangleIndex] = index;
                index = smallest;
            }

            _heap[index] = node;
            _heapPosition[node.TriangleIndex] = index;
        }
    }
}
