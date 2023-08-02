#pragma once
#include "common.h"

class TMatrix
{
public: 
    vector< vector<LL> > val;
    int n, m;
    inline TMatrix(){}
    inline TMatrix(int n, int m): n(n), m(m)
    {
        val.resize(n, vector<LL>(m,0) );
    }
    inline void setValue(int i, int j, LL v)
    {
        val[i][j] = v;
    }
    TMatrix operator * (const TMatrix &b)
    {
        TMatrix A(n, b.m);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
            {
                if (val[i][j] == 0) continue;
                for (int k = 0; k < b.m; k++)
                    A.val[i][k] += val[i][j] * b.val[j][k];
            }
        return A;
    }
};