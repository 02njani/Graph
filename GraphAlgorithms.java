import java.util.ArrayList;
import java.util.HashSet;
import java.util.HashMap;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Your implementation of various different graph algorithms.
 *
 * @author Nitya Jani
 * @version 1.0
 * @userid njani8
 * @GTID 903598748
 *
 * Collaborators: LIST ALL COLLABORATORS YOU WORKED WITH HERE
 *
 * Resources: LIST ALL NON-COURSE RESOURCES YOU CONSULTED HERE
 */
public class GraphAlgorithms {

    /**
     * Performs a breadth first search (bfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * You may import/use java.util.Set, java.util.List, java.util.Queue, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for BFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the bfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> bfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null || !graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("The vertex is not in the graph or it (or the graph) is null.");
        }
        Queue<Vertex<T>> q = new LinkedList<>();
        HashSet<Vertex<T>> visited = new HashSet<>();
        List<Vertex<T>> list = new ArrayList<>();
        q.add(start);
        while (!q.isEmpty()) {
            Vertex<T> x = q.remove();
            List<VertexDistance<T>> adjacent = graph.getAdjList().get(x);
            if (!visited.contains(x)) {
                visited.add(x);
                list.add(x);
                for (int i = 0; i < adjacent.size(); i++) {
                    if (!visited.contains(adjacent.get(i).getVertex())) {
                        q.add(adjacent.get(i).getVertex());
                    }
                }
            }
        }
        return list;
    }

    /**
     * Performs a depth first search (dfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * *NOTE* You MUST implement this method recursively, or else you will lose
     * all points for this method.
     *
     * You may import/use java.util.Set, java.util.List, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for DFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the dfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> dfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null || !graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("The vertex is not in the graph or the vertex or graph is null.");
        }
        HashSet<Vertex<T>> visited = new HashSet<>();
        List<Vertex<T>> list = new ArrayList<>();
        dfsHelper(start, visited, list, graph);
        return list;
    }

    /**
     * This is a helper method that is recursive.
     * @param start is the start vertex
     * @param visited is the set of visited vertices
     * @param list is the list to return
     * @param graph is the graph
     * @param <T> is the type
     */
    private static <T> void dfsHelper(Vertex<T> start, HashSet<Vertex<T>> visited,
                                      List<Vertex<T>> list, Graph<T> graph) {
        List<VertexDistance<T>> adjacent = graph.getAdjList().get(start);
        visited.add(start);
        list.add(start);
        if (adjacent.size() == 0) {
            return;
        }
        for (int j = 0; j < adjacent.size(); j++) {
            if (!visited.contains(adjacent.get(j).getVertex())) {
                dfsHelper(adjacent.get(j).getVertex(), visited, list, graph);
            }
        }
    }

    /**
     * Finds the single-source shortest distance between the start vertex and
     * all vertices given a weighted graph (you may assume non-negative edge
     * weights).
     *
     * Return a map of the shortest distances such that the key of each entry
     * is a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing
     * infinity) if no path exists.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Map, and java.util.Set and any class that
     * implements the aforementioned interfaces, as long as your use of it
     * is efficient as possible.
     *
     * You should implement the version of Dijkstra's where you use two
     * termination conditions in conjunction.
     *
     * 1) Check if all the vertices have been visited.
     * 2) Check if the PQ is empty yet.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the Dijkstra's on (source)
     * @param graph the graph we are applying Dijkstra's to
     * @return a map of the shortest distances from start to every
     * other node in the graph
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null || !graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("The graph or start vertex is null or the vertex is not in the graph.");
        }
        HashSet<Vertex<T>> visited = new HashSet<>();
        Map<Vertex<T>, Integer> distances = new HashMap<>();
        PriorityQueue<VertexDistance<T>> q = new PriorityQueue<>();
        for (Vertex<T> vertex : graph.getVertices()) {
            distances.put(vertex, Integer.MAX_VALUE);
        }
        q.add(new VertexDistance<>(start, 0));
        while (!q.isEmpty() && visited.size() != graph.getVertices().size()) {
            VertexDistance<T> d = q.remove();
            if (!visited.contains(d.getVertex())) {
                visited.add(d.getVertex());
                distances.put(d.getVertex(), d.getDistance());
                List<VertexDistance<T>> adjacent = graph.getAdjList().get(d.getVertex());
                for (int i = 0; i < adjacent.size(); i++) {
                    if (!visited.contains(adjacent.get(i).getVertex())) {
                        q.add(new VertexDistance<>(adjacent.get(i).getVertex(),
                                d.getDistance() + adjacent.get(i).getDistance()));
                    }
                }
            }
        }
        return distances;
    }

    /**
     * Runs Kruskal's algorithm on the given graph and returns the Minimal
     * Spanning Tree (MST) in the form of a set of Edges. If the graph is
     * disconnected and therefore no valid MST exists, return null.
     *
     * You may assume that the passed in graph is undirected. In this framework,
     * this means that if (u, v, 3) is in the graph, then the opposite edge
     * (v, u, 3) will also be in the graph, though as a separate Edge object.
     *
     * The returned set of edges should form an undirected graph. This means
     * that every time you add an edge to your return set, you should add the
     * reverse edge to the set as well. This is for testing purposes. This
     * reverse edge does not need to be the one from the graph itself; you can
     * just make a new edge object representing the reverse edge.
     *
     * You may assume that there will only be one valid MST that can be formed.
     *
     * Kruskal's will also require you to use a Disjoint Set which has been
     * provided for you. A Disjoint Set will keep track of which vertices are
     * connected given the edges in your current MST, allowing you to easily
     * figure out whether adding an edge will create a cycle. Refer
     * to the DisjointSet and DisjointSetNode classes that
     * have been provided to you for more information.
     *
     * You should NOT allow self-loops or parallel edges into the MST.
     *
     * By using the Disjoint Set provided, you can avoid adding self-loops and
     * parallel edges into the MST.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Set, and any class that implements the aforementioned
     * interfaces.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param graph the graph we are applying Kruskals to
     * @return the MST of the graph or null if there is no valid MST
     * @throws IllegalArgumentException if any input is null
     */
    public static <T> Set<Edge<T>> kruskals(Graph<T> graph) {
        if (graph == null) {
            throw new IllegalArgumentException("The graph is null.");
        }
        DisjointSet<Vertex<T>> disjoint = new DisjointSet<>();
        Set<Edge<T>> mst = new HashSet<>();
        PriorityQueue<Edge<T>> q = new PriorityQueue<>();
        for (Edge<T> e : graph.getEdges()) {
            q.add(e);
        }
        while (!q.isEmpty() && mst.size() < (graph.getVertices().size() - 1) * 2) {
            Edge<T> edge = q.remove();
            if (!disjoint.find(edge.getU()).equals(disjoint.find(edge.getV()))) {
                mst.add(edge);
                mst.add(new Edge<>(edge.getV(), edge.getU(), edge.getWeight()));
                disjoint.union(edge.getU(), edge.getV());
            }
        }
        if (mst.size() == (graph.getVertices().size() - 1) * 2) {
            return mst;
        } else {
            return null;
        }
    }
}
