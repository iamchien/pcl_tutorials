#include <bits/stdc++.h>

// #include <algorithm>
// #include <vector>
// #include <cmath>
// #include <iostream>

using namespace std;

typedef pair<double, double> point;

bool cw(const point &a, const point &b, const point &c) {
    return (b.first - a.first) * (c.second - a.second) - (b.second - a.second) * (c.first - a.first) < 0;
}

vector<point> convexHull(vector<point> p) {
    int n = p.size();
    if (n <= 1)
        return p;
    int k = 0;
    sort(p.begin(), p.end());
    vector<point> q(n * 2);
    for (int i = 0; i < n; q[k++] = p[i++])
        for (; k >= 2 && !cw(q[k - 2], q[k - 1], p[i]); --k)
            ;
    for (int i = n - 2, t = k; i >= 0; q[k++] = p[i--])
        for (; k > t && !cw(q[k - 2], q[k - 1], p[i]); --k)
            ;
    q.resize(k - 1 - (q[0] == q[1]));
    return q;
}

double area(const point &a, const point &b, const point &c) {
    return abs((b.first - a.first) * (c.second - a.second) - (b.second - a.second) * (c.first - a.first));
}

double dist(const point &a, const point &b) {
    return hypot(a.first - b.first, a.second - b.second);
}

double diameter(const vector<point> &p) {
    vector<point> h = convexHull(p);
    int m = h.size();
    std::cout << "Size of convexHull is: " << m << endl;
    for (auto it = h.begin(); it != h.end(); ++it){
        std::cout << it->first << " " << it->second << endl;
    }

    if (m == 1)
        return 0;
    if (m == 2)
        return dist(h[0], h[1]);
    int k = 1;
    while (area(h[m - 1], h[0], h[(k + 1) % m]) > area(h[m - 1], h[0], h[k]))
        ++k;
    
    std::cout << "k is: " << k << endl;

    double res = 0;
    for (int i = 0, j = k; i <= k && j < m; i++) {
        res = max(res, dist(h[i], h[j]));
        while (j < m && area(h[i], h[(i + 1) % m], h[(j + 1) % m]) > area(h[i], h[(i + 1) % m], h[j])) {
            res = max(res, dist(h[i], h[(j + 1) % m]));
            ++j;
        }
    }
    return res;
}

int main() {
    vector<point> points{{ 0, 0},{ 3, 0},{ 0, 3},
                         { 1, 1},{ 3, 4},{ 4, 2},
                         {-2, 2},{ 2,-1},{ 2, 6},
                         {-4, 1},{ 4,-2},{ 0,-2},
                         { 6,-2},{-4, 4},{-2, 6}};
    double d = diameter(points);
    std::cout << d << endl;
}