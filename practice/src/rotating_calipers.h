#include <bits/stdc++.h>

using namespace std;

typedef pair<double, double> point;

bool cw(const point &a, const point &b, const point &c);
vector<point> convexHull(vector<point> p);
double area(const point &a, const point &b, const point &c);
double dist(const point &a, const point &b);
double diameter(const vector<point> &p);


