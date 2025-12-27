using System.Collections.Generic;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// A*용 Indexed Min-Heap (배열 기반 최적화)
    /// Dictionary 대신 배열 + Generation 트릭 사용
    /// </summary>
    public class IndexedMinHeap
    {
        private struct HeapNode
        {
            public int TriangleIndex;
            public long GScore;
            public long HScore;
            public long FScore => GScore + HScore;

            public HeapNode(int triangleIndex, long g, long h)
            {
                TriangleIndex = triangleIndex;
                GScore = g;
                HScore = h;
            }
        }

        private readonly HeapNode[] _heap;
        private readonly int[] _heapPosition;      // triangleIndex → heap position
        private readonly int[] _positionGeneration; // 해당 position이 유효한 generation
        private int _heapCount;
        private int _generation;
        private readonly int _maxCapacity;

        public int Count => _heapCount;

        public IndexedMinHeap(int maxTriangleCount)
        {
            _maxCapacity = maxTriangleCount;
            _heap = new HeapNode[maxTriangleCount];
            _heapPosition = new int[maxTriangleCount];
            _positionGeneration = new int[maxTriangleCount];
            _heapCount = 0;
            _generation = 0;
        }

        public void Clear()
        {
            _heapCount = 0;
            _generation++;

            // Generation 오버플로우 체크
            if (_generation == int.MaxValue)
            {
                _generation = 1;
                System.Array.Clear(_positionGeneration, 0, _positionGeneration.Length);
            }
        }

        /// <summary>
        /// 삼각형이 힙에 있는지 확인 - O(1)
        /// </summary>
        public bool Contains(int triangleIndex)
        {
            return _positionGeneration[triangleIndex] == _generation;
        }

        /// <summary>
        /// 새 노드 추가 - O(log n)
        /// </summary>
        public void Insert(int triangleIndex, long g, long h)
        {
            int index = _heapCount;
            _heap[index] = new HeapNode(triangleIndex, g, h);
            _heapPosition[triangleIndex] = index;
            _positionGeneration[triangleIndex] = _generation;
            _heapCount++;
            SiftUp(index);
        }

        /// <summary>
        /// 최소 F값 노드 제거 및 반환 - O(log n)
        /// </summary>
        public (int TriangleIndex, long GScore, long HScore) ExtractMin()
        {
            HeapNode min = _heap[0];

            _heapCount--;
            if (_heapCount > 0)
            {
                _heap[0] = _heap[_heapCount];
                _heapPosition[_heap[0].TriangleIndex] = 0;
                SiftDown(0);
            }

            // min은 더 이상 힙에 없음 (generation 무효화)
            _positionGeneration[min.TriangleIndex] = 0;

            return (min.TriangleIndex, min.GScore, min.HScore);
        }

        /// <summary>
        /// 기존 노드의 우선순위 업데이트 - O(log n)
        /// </summary>
        public void UpdatePriority(int triangleIndex, long newG, long newH)
        {
            if (_positionGeneration[triangleIndex] != _generation)
                return;

            int index = _heapPosition[triangleIndex];
            long oldF = _heap[index].FScore;
            _heap[index] = new HeapNode(triangleIndex, newG, newH);
            long newF = _heap[index].FScore;

            if (newF < oldF)
            {
                SiftUp(index);
            }
            else if (newF > oldF)
            {
                SiftDown(index);
            }
        }

        private void SiftUp(int index)
        {
            while (index > 0)
            {
                int parent = (index - 1) / 2;
                if (_heap[index].FScore >= _heap[parent].FScore)
                    break;

                Swap(index, parent);
                index = parent;
            }
        }

        private void SiftDown(int index)
        {
            while (true)
            {
                int smallest = index;
                int left = 2 * index + 1;
                int right = 2 * index + 2;

                if (left < _heapCount && _heap[left].FScore < _heap[smallest].FScore)
                    smallest = left;

                if (right < _heapCount && _heap[right].FScore < _heap[smallest].FScore)
                    smallest = right;

                if (smallest == index)
                    break;

                Swap(index, smallest);
                index = smallest;
            }
        }

        private void Swap(int a, int b)
        {
            HeapNode temp = _heap[a];
            _heap[a] = _heap[b];
            _heap[b] = temp;

            _heapPosition[_heap[a].TriangleIndex] = a;
            _heapPosition[_heap[b].TriangleIndex] = b;
        }
    }
}
