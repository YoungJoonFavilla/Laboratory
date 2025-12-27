using System.Collections.Generic;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// A*용 Indexed Min-Heap (raw long 최적화)
    /// F값 기준 최소 힙 + 삼각형 인덱스로 O(1) 조회
    /// </summary>
    public class IndexedMinHeap
    {
        private struct HeapNode
        {
            public int TriangleIndex;
            public long GScore;  // raw Fixed64 값
            public long HScore;  // raw Fixed64 값
            public long FScore => GScore + HScore;

            public HeapNode(int triangleIndex, long g, long h)
            {
                TriangleIndex = triangleIndex;
                GScore = g;
                HScore = h;
            }
        }

        private readonly List<HeapNode> _heap = new List<HeapNode>();
        private readonly Dictionary<int, int> _indexMap = new Dictionary<int, int>();

        public int Count => _heap.Count;

        public void Clear()
        {
            _heap.Clear();
            _indexMap.Clear();
        }

        /// <summary>
        /// 삼각형이 힙에 있는지 확인 - O(1)
        /// </summary>
        public bool Contains(int triangleIndex)
        {
            return _indexMap.ContainsKey(triangleIndex);
        }

        /// <summary>
        /// 새 노드 추가 - O(log n)
        /// </summary>
        public void Insert(int triangleIndex, long g, long h)
        {
            HeapNode node = new HeapNode(triangleIndex, g, h);
            int index = _heap.Count;
            _heap.Add(node);
            _indexMap[triangleIndex] = index;
            SiftUp(index);
        }

        /// <summary>
        /// 최소 F값 노드 제거 및 반환 - O(log n)
        /// </summary>
        public (int TriangleIndex, long GScore, long HScore) ExtractMin()
        {
            HeapNode min = _heap[0];

            int lastIndex = _heap.Count - 1;
            if (lastIndex > 0)
            {
                _heap[0] = _heap[lastIndex];
                _indexMap[_heap[0].TriangleIndex] = 0;
            }

            _heap.RemoveAt(lastIndex);
            _indexMap.Remove(min.TriangleIndex);

            if (_heap.Count > 0)
            {
                SiftDown(0);
            }

            return (min.TriangleIndex, min.GScore, min.HScore);
        }

        /// <summary>
        /// 기존 노드의 우선순위 업데이트 - O(log n)
        /// </summary>
        public void UpdatePriority(int triangleIndex, long newG, long newH)
        {
            if (!_indexMap.TryGetValue(triangleIndex, out int index))
                return;

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
            int count = _heap.Count;
            while (true)
            {
                int smallest = index;
                int left = 2 * index + 1;
                int right = 2 * index + 2;

                if (left < count && _heap[left].FScore < _heap[smallest].FScore)
                    smallest = left;

                if (right < count && _heap[right].FScore < _heap[smallest].FScore)
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

            _indexMap[_heap[a].TriangleIndex] = a;
            _indexMap[_heap[b].TriangleIndex] = b;
        }
    }
}
