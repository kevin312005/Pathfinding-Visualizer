#include <bits/stdc++.h>
using namespace std;

struct Point {
    int x, y;
    Point(int _x=-1, int _y=-1) { x=_x; y=_y; }
};

// arah gerakan: atas, bawah, kiri, kanan
int dx[4] = {-1, 1, 0, 0};
int dy[4] = {0, 0, -1, 1};

// rekonstruksi jalur dari parent
void reconstructPath(vector<string> &grid, Point start, Point goal,
                     vector<vector<Point> > &parent) {
    Point p = goal;
    while (!(p.x == start.x && p.y == start.y)) {
        if (grid[p.x][p.y] != 'G') grid[p.x][p.y] = '*';
        p = parent[p.x][p.y];
    }
}

// BFS
bool bfs(vector<string> &grid, Point start, Point goal) {
    int n = grid.size(), m = grid[0].size();
    vector<vector<bool> > visited(n, vector<bool>(m, false));
    vector<vector<Point> > parent(n, vector<Point>(m, Point(-1,-1)));

    queue<Point> q;
    q.push(start);
    visited[start.x][start.y] = true;

    while (!q.empty()) {
        Point cur = q.front(); q.pop();
        if (cur.x == goal.x && cur.y == goal.y) {
            reconstructPath(grid, start, goal, parent);
            return true;
        }
        for (int i = 0; i < 4; i++) {
            int nx = cur.x + dx[i], ny = cur.y + dy[i];
            if (nx>=0 && nx<n && ny>=0 && ny<m &&
                grid[nx][ny] != '#' && !visited[nx][ny]) {
                visited[nx][ny] = true;
                parent[nx][ny] = cur;
                q.push(Point(nx,ny));
            }
        }
    }
    return false;
}

// DFS
bool dfsUtil(vector<string> &grid, Point cur, Point goal,
             vector<vector<bool> > &visited,
             vector<vector<Point> > &parent) {
    if (cur.x == goal.x && cur.y == goal.y) return true;

    visited[cur.x][cur.y] = true;
    for (int i = 0; i < 4; i++) {
        int nx = cur.x + dx[i], ny = cur.y + dy[i];
        if (nx>=0 && nx<grid.size() && ny>=0 && ny<grid[0].size() &&
            grid[nx][ny] != '#' && !visited[nx][ny]) {
            parent[nx][ny] = cur;
            if (dfsUtil(grid, Point(nx,ny), goal, visited, parent))
                return true;
        }
    }
    return false;
}

bool dfs(vector<string> &grid, Point start, Point goal) {
    int n = grid.size(), m = grid[0].size();
    vector<vector<bool> > visited(n, vector<bool>(m, false));
    vector<vector<Point> > parent(n, vector<Point>(m, Point(-1,-1)));

    if (dfsUtil(grid, start, goal, visited, parent)) {
        reconstructPath(grid, start, goal, parent);
        return true;
    }
    return false;
}

// Dijkstra
struct Node {
    int x, y, dist;
    Node(int _x, int _y, int _d) { x=_x; y=_y; dist=_d; }
    bool operator<(const Node &other) const {
        return dist > other.dist; // priority queue min-heap
    }
};

bool dijkstra(vector<string> &grid, Point start, Point goal) {
    int n = grid.size(), m = grid[0].size();
    vector<vector<int> > dist(n, vector<int>(m, 1e9));
    vector<vector<Point> > parent(n, vector<Point>(m, Point(-1,-1)));

    priority_queue<Node> pq;
    pq.push(Node(start.x, start.y, 0));
    dist[start.x][start.y] = 0;

    while (!pq.empty()) {
        Node cur = pq.top(); pq.pop();
        if (cur.x == goal.x && cur.y == goal.y) {
            reconstructPath(grid, start, goal, parent);
            return true;
        }
        for (int i = 0; i < 4; i++) {
            int nx = cur.x + dx[i], ny = cur.y + dy[i];
            if (nx>=0 && nx<n && ny>=0 && ny<m && grid[nx][ny] != '#') {
                int nd = cur.dist + 1; // bobot semua = 1
                if (nd < dist[nx][ny]) {
                    dist[nx][ny] = nd;
                    parent[nx][ny] = Point(cur.x, cur.y);
                    pq.push(Node(nx, ny, nd));
                }
            }
        }
    }
    return false;
}

int main() {
    int n, m;
    cout << "Masukkan ukuran grid (baris kolom): ";
    cin >> n >> m;

    vector<string> grid(n);
    cout << "Masukkan grid (S = start, G = goal, # = tembok, . = jalan kosong):\n";
    for (int i = 0; i < n; i++) {
        cin >> grid[i];
    }

    Point start, goal;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (grid[i][j] == 'S') start = Point(i, j);
            if (grid[i][j] == 'G') goal = Point(i, j);
        }
    }

    int choice;
    cout << "\nPilih algoritma:\n";
    cout << "1. BFS\n2. DFS\n3. Dijkstra\n";
    cout << "Pilihan: ";
    cin >> choice;

    bool found = false;
    if (choice == 1) found = bfs(grid, start, goal);
    else if (choice == 2) found = dfs(grid, start, goal);
    else if (choice == 3) found = dijkstra(grid, start, goal);

    if (found) cout << "\nPath found:\n";
    else cout << "\nNo path exists!\n";

    for (int i = 0; i < n; i++) {
        cout << grid[i] << "\n";
    }

    system("pause");
    return 0;
}

