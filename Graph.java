

import java.util.*;

/**
 * @program: 
 * @description:
 * @author: dyingstraw
 * @create: 2019-03-29 21:20
 **/
public class Graph {
    public HashMap<Integer,Node> nodes = new HashMap<>();
    public HashSet<Edge> edges = new HashSet<>();
    private int[][] maxtix;
    private int[][] p;


    // 将一个节点加入图
    public void add2Graph(Integer from,Integer to,Integer weight){
        if (!nodes.containsKey(from)){
            nodes.put(from,new Node(from));
        }
        if (!nodes.containsKey(to)){
            nodes.put(to,new Node(to));
        }
        Node fromNode = nodes.get(from);
        Node toNode = nodes.get(to);
        Edge newEdge = new Edge(from, to, weight);
        fromNode.out++;
        toNode.in++;
        fromNode.nexts.add(toNode);
        fromNode.edges.add(newEdge);
        edges.add(newEdge);
    }
    // 将一个节点加入图
    public void add2Graph(Integer from,Integer to,Integer weight,Integer max){
        if (!nodes.containsKey(from)){
            nodes.put(from,new Node(from));
        }
        if (!nodes.containsKey(to)){
            nodes.put(to,new Node(to));
        }
        Node fromNode = nodes.get(from);
        Node toNode = nodes.get(to);
        Edge newEdge = new Edge(from, to, weight,max);
        fromNode.out++;
        toNode.in++;
        fromNode.nexts.add(toNode);
        fromNode.edges.add(newEdge);
        edges.add(newEdge);
    }
    // 宽度优先遍历
    public void wfs(Node node){
        if (node==null)return;
        Queue<Node> queue = new LinkedList<>();
        HashSet<Node> set = new HashSet<>();
        queue.add(node);
        set.add(node);
        while (!queue.isEmpty()){
            Node cur = queue.poll();
            System.out.print(cur.value+" ");
            for (Node next:cur.nexts){
                if(!set.contains(next)){
                    set.add(next);
                    queue.add(next);

                }
            }
        }
    }
    // 深度优先 栈实现
    public void dfs(Node node){
        if (node==null)return;
        Stack<Node> stack = new Stack<>();
        HashSet<Node> set = new HashSet<>();
        stack.push(node);
        set.add(node);
        System.out.print(node.value+" ");
        while(!stack.empty()){
            Node cur = stack.pop();
            for (Node next:cur.nexts) {
                if (!set.contains(next)){
                    stack.push(cur);
                    stack.push(next);
                    set.add(next);
                    System.out.print(next.value+" ");
                    break;
                }
            }
        }
    }
    // 拓扑排序
    public List<Node> topologySort(){
        HashMap<Node,Integer> temp = new HashMap<>();
        Queue<Node> queue = new LinkedList<Node>();
        for (Node node:nodes.values()){
            temp.put(node,node.in);
            if (node.in==0){
                ((LinkedList<Node>) queue).push(node);
            }

        }
        List<Node>result = new ArrayList<>();
        while (!queue.isEmpty()){
            // 更新下个节点的入度
            Node cur = ((LinkedList<Node>) queue).pop();
            result.add(cur);

            for (Node next : cur.nexts){
                temp.put(next,temp.get(next)-1);
                if (temp.get(next)==0){
                    ((LinkedList<Node>) queue).push(next);
                }
            }
        }
        return result;
    }

    public void kruskalMST(){

    }
    public void prim(Node node){
        Set<Node> visitedNodes = new HashSet<>();
        Set<Edge> visitedEdges = new HashSet<>();
        PriorityQueue<Edge> unVisitedEdges = new PriorityQueue<>(new Comparator<Edge>() {
            @Override
            public int compare(Edge o1, Edge o2) {
                return o1.weight-o2.weight;
            }
        });
        visitedNodes.add(node);

        while(visitedNodes.size()<nodes.size()){
            for (Edge e:node.edges){
                unVisitedEdges.add(e);
            }
            Edge cur = unVisitedEdges.poll();
            while (!visitedEdges.contains(cur)){
                node = nodes.get(cur.to);
                visitedEdges.add(cur);
                visitedNodes.add(node);
                System.out.println(cur.from+"-->"+cur.to + "  ");
                break;
            }
        }
    }
    public HashMap<Node, Integer> dijkstra(Node head, int size) {
        NodeHeap nodeHeap = new NodeHeap(size);
        nodeHeap.addOrUpdateOrIgnore(head, 0);
        HashMap<Node, Integer> result = new HashMap<>();

        while (!nodeHeap.isEmpty()) {
            NodeRecord record = nodeHeap.pop();
            Node cur = record.node;
            int distance = record.distance;
            for (Edge edge : cur.edges) {
                nodeHeap.addOrUpdateOrIgnore(nodes.get(edge.to), edge.weight + distance);
            }
            result.put(cur, distance);
        }
        return result;
    }

    public HashMap<Node, ArrayList<Node>> dijkstra2(Node head, int max) {
        return null;
    }
    public void floyd(){
        int[][] matix= new int[nodes.size()][nodes.size()];
        int[][] p= new int[nodes.size()][nodes.size()];
        for (int i = 0; i <nodes.size()  ; i++) {
            for (int j = 0; j <nodes.size(); j++) {
                if (i==j) matix[i][j]=0;
                else matix[i][j]=999999;
            }
        }

        for (Edge e:edges) {
            matix[e.from][e.to] = e.weight;
            matix[e.to][e.from] = e.weight;
        }
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < nodes.size(); j++) {

                p[i][j] =j;
            }
        }

        for (int i = 0; i < nodes.size(); i++) {
           System.out.println(Arrays.toString(matix[i]));
        }

        for (int k = 0; k < nodes.size(); k++) {
            for (int v = 0; v < nodes.size() ; v++) {
                for (int w = 0; w < nodes.size(); w++) {
                    if (matix[v][w] > (matix[v][k] + matix[k][w])){
                        matix[v][w] = (matix[v][k] + matix[k][w]);
                        p[v][w] = p[v][k];
                     }
                }
            }
        }
        this.maxtix =matix;
        this.p =p;
    }
    public List<Integer> getMinPath(int from,int to){
        ArrayList<Integer> re = new ArrayList<>();
        re.add(maxtix[from][to]);
        if (maxtix[from][to]!=999999){
            re.add(from);
            int temp = p[from][to];
            while (temp!=to){
                re.add(temp);
                temp =p[temp][to];
            }
            re.add(to);

        }

        // for (int i = 0; i < nodes.size(); i++) {
        //     for (int j = 0; j < nodes.size() ; j++) {
        //         if (this.maxtix[i][j]!=999999){
        //             System.out.printf("%d到%d的最短路径长度为%d\n",i,j,maxtix[i][j]);
        //             int temp = p[i][j];
        //             while (temp!=j){
        //                 System.out.print(temp+" ");
        //                 temp = p[temp][j];
        //             }
        //             System.out.println(j+" ");
        //         }
        //         System.out.println("");
        //     }
        //     System.out.println("");
        // }
        return re;
    }



    public static void main(String[] args){
        Graph graph = new Graph();
        graph.add2Graph(0,1,6);
        graph.add2Graph(0,4,2);
        graph.add2Graph(1,2,1);
        graph.add2Graph(1,3,3);
        graph.add2Graph(1,4,3);
        graph.add2Graph(2,3,1);
        graph.add2Graph(2,7,2);
        graph.add2Graph(3,5,4);
        graph.add2Graph(4,6,1);
        graph.add2Graph(7,3,1);
        graph.add2Graph(5,6,1);
        graph.add2Graph(1,6,2);

        System.out.println(graph.nodes.get(1).nexts);
        graph.wfs(graph.nodes.get(1));
        System.out.println("");
        graph.dfs(graph.nodes.get(1));
        System.out.println("");
        List<Node> r = graph.topologySort();
        System.out.println(r);
        System.out.println("");
        // graph.prim(graph.nodes.get(0));

        graph.floyd();
        List<Integer> s = graph.getMinPath(1, 5);
        System.out.println(s);
    }
    public static class NodeRecord {
        public Node node;
        public int distance;
        public int count=0;

        public NodeRecord(Node node, int distance) {
            this.node = node;
            this.distance = distance;
        }
        public NodeRecord(Node node, int distance,int count) {
            this.node = node;
            this.distance = distance;
            this.count =count;
        }
    }
    public static class NodeHeap {
        private Node[] nodes;
        private HashMap<Node, Integer> heapIndexMap;
        private HashMap<Node, Integer> distanceMap;
        private int size;

        public NodeHeap(int size) {
            nodes = new Node[size];
            heapIndexMap = new HashMap<>();
            distanceMap = new HashMap<>();
            this.size = 0;
        }

        public boolean isEmpty() {
            return size == 0;
        }

        public void addOrUpdateOrIgnore(Node node, int distance) {
            if (inHeap(node)) {
                distanceMap.put(node, Math.min(distanceMap.get(node), distance));
                insertHeapify(node, heapIndexMap.get(node));
            }
            if (!isEntered(node)) {
                nodes[size] = node;
                heapIndexMap.put(node, size);
                distanceMap.put(node, distance);
                insertHeapify(node, size++);
            }
        }


    public NodeRecord pop() {
            NodeRecord nodeRecord = new NodeRecord(nodes[0], distanceMap.get(nodes[0]));
            swap(0, size - 1);
            heapIndexMap.put(nodes[size - 1], -1);
            distanceMap.remove(nodes[size - 1]);
            nodes[size - 1] = null;
            heapify(0, --size);
            return nodeRecord;
        }

        private void insertHeapify(Node node, int index) {
            while (distanceMap.get(nodes[index]) < distanceMap.get(nodes[(index - 1) / 2])) {
                swap(index, (index - 1) / 2);
                index = (index - 1) / 2;
            }
        }

        private void heapify(int index, int size) {
            int left = index * 2 + 1;
            while (left < size) {
                int smallest = left + 1 < size && distanceMap.get(nodes[left + 1]) < distanceMap.get(nodes[left])
                        ? left + 1 : left;
                smallest = distanceMap.get(nodes[smallest]) < distanceMap.get(nodes[index]) ? smallest : index;
                if (smallest == index) {
                    break;
                }
                swap(smallest, index);
                index = smallest;
                left = index * 2 + 1;
            }
        }

        private boolean isEntered(Node node) {
            return heapIndexMap.containsKey(node);
        }

        private boolean inHeap(Node node) {
            return isEntered(node) && heapIndexMap.get(node) != -1;
        }

        private void swap(int index1, int index2) {
            heapIndexMap.put(nodes[index1], index2);
            heapIndexMap.put(nodes[index2], index1);
            Node tmp = nodes[index1];
            nodes[index1] = nodes[index2];
            nodes[index2] = tmp;
        }
    }










    class Node{
        public int value;
        public int in;
        public int out;
        // 所有的邻居节点
        public ArrayList<Node> nexts;
        // 从我出发的边
        public ArrayList<Edge> edges;

        public Node(int value) {
            this.value = value;
            in=0;
            out=0;
            nexts = new ArrayList<>();
            edges = new ArrayList<>();
        }

        @Override
        public String toString() {
            return String.format("%d",value);
        }
    }
    class Edge{
        public int from;
        public int to;
        public int weight;
        public int max;
        public Edge(int from, int to, int weight) {
            this.from = from;
            this.to = to;
            this.weight = weight;
        }

        public Edge(int from, int to, int weight,int max) {
            this.from = from;
            this.to = to;
            this.weight = weight;
            this.max = max;
        }
    }

}
