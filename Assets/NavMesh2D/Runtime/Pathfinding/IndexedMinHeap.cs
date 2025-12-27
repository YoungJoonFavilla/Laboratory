using System.Collections.Generic;
using FixedMathSharp;

namespace NavMesh2D.Pathfinding
{
    /// <summary>
    /// A*용 Indexed Min-Heap
    /// F값 기준 최소 힙 + 삼각형 인덱스로 O(1) 조회
    /// </summary>
    public class IndexedMinHeap
    {
        private struct HeapNode
        {
            public int TriangleIndex;
            public Fixed64 GScore;
            public Fixed64 HScore;
            public Fixed64 FScore => GScore + HScore;

            public HeapNode(int triangleIndex, Fixed64 g, Fixed64 h)
            {
                TriangleIndex = triangleIndex;
                GScore = g;
                HScore = h;
            }
        }

        private readonly List<HeapNode> _heap = new List<HeapNode>();
        private readonly Dictionary<int, int> _indexMap = new Dictionary<int, int>(); // 삼각형번호 → 힙위치

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
        public void Insert(int triangleIndex, Fixed64 g, Fixed64 h)
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
        public (int TriangleIndex, Fixed64 GScore, Fixed64 HScore) ExtractMin()
        {
            HeapNode min = _heap[0];

            // 마지막 노드를 루트로 이동
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
        /// 기존 노드의 G값 업데이트 (DecreaseKey) - O(log n)
        /// </summary>
        public void UpdatePriority(int triangleIndex, Fixed64 newG, Fixed64 newH)
        {
            if (!_indexMap.TryGetValue(triangleIndex, out int index))
                return;

            Fixed64 oldF = _heap[index].FScore;
            _heap[index] = new HeapNode(triangleIndex, newG, newH);
            Fixed64 newF = _heap[index].FScore;

            // F가 감소했으면 위로, 증가했으면 아래로
            if (newF < oldF)
            {
                SiftUp(index);
            }
            else if (newF > oldF)
            {
                SiftDown(index);
            }
        }

        /// <summary>
        /// 위로 정렬 (부모와 비교)
        /// </summary>
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

        /// <summary>
        /// 아래로 정렬 (자식과 비교)
        /// </summary>
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

        /// <summary>
        /// 두 노드 위치 교환
        /// </summary>
        private void Swap(int a, int b)
        {
            HeapNode temp = _heap[a];
            _heap[a] = _heap[b];
            _heap[b] = temp;

            // 인덱스 맵 동기화
            _indexMap[_heap[a].TriangleIndex] = a;
            _indexMap[_heap[b].TriangleIndex] = b;
        }
    }
}
