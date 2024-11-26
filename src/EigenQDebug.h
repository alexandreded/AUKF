// EigenQDebug.h
#ifndef EIGENQDEBUG_H
#define EIGENQDEBUG_H

#include <QDebug>
#include <Eigen/Dense>

// Перегрузка оператора << для QDebug и Eigen::VectorXd
inline QDebug operator<<(QDebug dbg, const Eigen::VectorXd& vec)
{
    dbg.nospace() << "[";
    for(int i = 0; i < vec.size(); ++i){
        dbg.nospace() << vec[i];
        if(i != vec.size()-1)
            dbg.nospace() << ", ";
    }
    dbg.nospace() << "]";
    return dbg.space();
}

#endif // EIGENQDEBUG_H
