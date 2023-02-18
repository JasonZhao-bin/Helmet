#include "tag_pos.h"
/*
 *	功能：根据基站的坐标值，判断是否有效三角形
 *
 */
bool is_valid_Anccoord(double x1, double y1, double x2, double y2, double x3, double y3)
{
    bool ret = false;

    double dist12, dist13, dist23;

    dist12 = sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
    dist13 = sqrt(pow(x1-x3, 2) + pow(y1-y3, 2));
    dist23 = sqrt(pow(x2-x3, 2) + pow(y2-y3, 2));


    if((dist12 > (dist13 + dist23)) &&
       (dist13 > (dist12 + dist23)) &&
       (dist23 > (dist12 + dist13)) )
    {
        ret = true;
    }

    return ret;
}


/*
 *	一维定位算法，按照基站坐标画圆，相较于一点或者两点
 */



// 浮点数判同
int double_equals(double const a, double const b)
{
    static const double ZERO = 1e-9;
    return fabs(a - b) < ZERO;
}

// 两点之间距离的平方
double distance_sqr(struct cross_point_t const* a, struct cross_point_t const* b)
{
    return (a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y);
}

// 两点之间的距离
double distance(struct cross_point_t const* a, struct cross_point_t const* b)
{
    return sqrt(distance_sqr(a, b));
}

/*
* 两圆相交函数
* 参数:
*    circles[0] 和 circles[1] 分别是两个圆.
*    points[0] 和 points[1] 用来存放交点数值, 虽然有些情况下两个不都会用上;
*        如果用到了两个交点, 那么返回后, 横坐标大的在前, 如果横坐标一样, 则纵坐标大的在前.
* 返回值:
*    -1 如果两个圆一模一样;
*    其它整数值: 交点个数.
*/
int Get_Cross_circle(struct circle_t circles[], struct cross_point_t points[])
{
    double d, a, b, c, p, q, r; // a, b, c, p, q, r 与上面分析中的量一致
    double cos_value[2], sin_value[2]; // 交点的在 circles[0] 上对应的正余弦取值
                                       // 余弦值 cos_value 就是分析中的 cosθ
    if (double_equals(circles[0].center.x, circles[1].center.x)
        && double_equals(circles[0].center.y, circles[1].center.y)
        && double_equals(circles[0].r, circles[1].r)) {
        return CIRCLE_CROSS_INVALID;
    }

    d = distance(&circles[0].center, &circles[1].center); // 圆心距离
    if (d > circles[0].r + circles[1].r
        || d < fabs(circles[0].r - circles[1].r)) {
        return CIRCLE_CROSS_NONE;
    }

    a = 2.0 * circles[0].r * (circles[0].center.x - circles[1].center.x);
    b = 2.0 * circles[0].r * (circles[0].center.y - circles[1].center.y);
    c = circles[1].r * circles[1].r - circles[0].r * circles[0].r
        - distance_sqr(&circles[0].center, &circles[1].center);
    p = a * a + b * b;
    q = -2.0 * a * c;

    // 如果交点仅一个
    if (double_equals(d, circles[0].r + circles[1].r)
        || double_equals(d, fabs(circles[0].r - circles[1].r))) {
        cos_value[0] = -q / p / 2.0;
        sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);

        points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
        points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;

        // 在这里验证解是否正确, 如果不正确, 则将纵坐标符号进行变换
        if(!double_equals(distance_sqr(&points[0], &circles[1].center),
                          circles[1].r * circles[1].r)) {
            points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
        }
        return CIRCLE_CROSS_ONE;
    }

    r = c * c - b * b;
    cos_value[0] = (sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
    cos_value[1] = (-sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
    sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);
    sin_value[1] = sqrt(1 - cos_value[1] * cos_value[1]);

    points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
    points[1].x = circles[0].r * cos_value[1] + circles[0].center.x;
    points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;
    points[1].y = circles[0].r * sin_value[1] + circles[0].center.y;

    // 验证解是否正确, 两个解都需要验证.
    if (!double_equals(distance_sqr(&points[0], &circles[1].center),
                       circles[1].r * circles[1].r)) {
        points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
    }
    if (!double_equals(distance_sqr(&points[1], &circles[1].center),
                       circles[1].r * circles[1].r)) {
        points[1].y = circles[0].center.y - circles[0].r * sin_value[1];
    }
    // 如果求得的两个点坐标相同, 则必然其中一个点的纵坐标反号可以求得另一点坐标
    if (double_equals(points[0].y, points[1].y)
        && double_equals(points[0].x, points[1].x)) {
        if(points[0].y > 0) {
            points[1].y = -points[1].y;
        } else {
            points[0].y = -points[0].y;
        }
    }
    return CIRCLE_CROSS_TWO;
}


int calculateTagCircle(vec3d_t *report, double x1, double y1, double d1, double x2, double y2, double d2)
{
    struct circle_t circle[2];
    struct cross_point_t cross_point[2];
    int ret;
    circle[0].center.x = x1;
    circle[0].center.y = y1;
    circle[0].r = d1;

    circle[1].center.x = x2;
    circle[1].center.y = y2;
    circle[1].r = d2;

    ret = Get_Cross_circle(circle, cross_point);
    switch(ret)
    {

        case CIRCLE_CROSS_ONE:
                if(circle[0].center.x == circle[1].center.x)		//x轴相同
                {
                    report->x = circle[0].center.x;
                    report->y = cross_point[0].y;
                    report->z = 0.0;
                }
                else if(circle[0].center.y == circle[1].center.y)	//y轴相同
                {
                    report->x = cross_point[0].x;
                    report->y = circle[0].center.y;
                    report->z = 0.0;
                }
                else //都不相等
                {
                    report->x = cross_point[0].x;
                    report->y = cross_point[0].y;
                    report->z = 0.0;
                }
            break;
        case CIRCLE_CROSS_TWO:
                if(circle[0].center.x == circle[1].center.x)	//x轴相同
                {
                    report->x = circle[0].center.x;
                    report->y = (cross_point[0].y + cross_point[1].y)/2.0;
                    report->z = 0.0;
                }
                else if(circle[0].center.y == circle[1].center.y) //y轴相同
                {
                    report->x = (cross_point[0].x + cross_point[1].x)/2.0;
                    report->y = circle[0].center.y;
                    report->z = 0.0;
                }
                else //都不相等
                {
                    report->x = (cross_point[0].x + cross_point[1].x)/2.0;
                    report->y = (cross_point[0].y + cross_point[1].y)/2.0;
                    report->z = 0.0;
                }
            break;
        case CIRCLE_CROSS_INVALID:
        case CIRCLE_CROSS_NONE:
        default:
            break;
    }

    return ret;
}




/*
 *	功能：根据三边算法解算一维坐标
 *
 */
bool Get_trilateration_1Dimen(double x1, double y1, double d1,
                            double x2, double y2, double d2,
                            vec3d_t *report)
{
    bool ret = false;

    int cross_num = calculateTagCircle(report, x1, y1, d1, x2, y2, d2);
    if(cross_num == CIRCLE_CROSS_ONE || cross_num == CIRCLE_CROSS_TWO)
    {
        //一维坐标解算成功
        ret = true;
    }
    else
    {
        //一维坐标解算失败
    }


    return ret;
}


/*
 *	功能：根据三边算法解算二维坐标(求解二元一次方程组 矩阵)
 *
 */
bool Get_trilateration_2Dimen(double x1, double y1, double d1,
                              double x2, double y2, double d2,
                              double x3, double y3, double d3,
                              vec3d_t *report)
{
//	if(is_valid_Anccoord(x1, y1, x2, y2, x3, y3) == false)
//	{
//		_dbg_printf("不符合的基站坐标\n");
//		return false;
//	}
//
    double a1 = 2 * (x1 - x3);
    double b1 = 2 * (y1 - y3);
    double r1 = pow(x1, 2) - pow(x3, 2) + pow(y1, 2) - pow(y3, 2) + pow(d3, 2) - pow(d1, 2);
    double a2 = 2 * (x2 - x3);
    double b2 = 2 * (y2 - y3);
    double r2 = pow(x2, 2) - pow(x3, 2) + pow(y2, 2) - pow(y3, 2) + pow(d3, 2) - pow(d2, 2);

    double det = a1 * b2 - a2 * b1;

    //检测分母是否为0,如果为零则退出
    if(det == 0.0)
    {
        return false;
    }

    report->x = (r1 * b2 - r2 * b1) / det;
    report->y = (r2 * a1 - r1 * a2) / det;

    return true;
}





/*
 *	功能：根据三边算法解算三维坐标(求解三元一次方程组 矩阵)
 *
 */
bool Get_trilateration_3Dimen(double x1, double y1, double z1, double d1,
                              double x2, double y2, double z2, double d2,
                              double x3, double y3, double z3, double d3,
                              double x4, double y4, double z4, double d4,
                              vec3d_t *report)
{
    //三元一次方程组变量
    double a1, b1, c1, r1;
    double a2, b2, c2, r2;
    double a3, b3, c3, r3;
    double a4, b4, c4, r4;

    a1 = 2*(x1-x4);
    b1 = 2*(y1-y4);
    c1 = 2*(z1-z4);
    r1 = pow(d4,2) - pow(d1,2) + pow(x1,2) - pow(x4,2) + pow(y1,2) - pow(y4,2) + pow(z1,2) - pow(z4,2);

    a2 = 2*(x2-x4);
    b2 = 2*(y2-y4);
    c2 = 2*(z2-z4);
    r2 = pow(d4,2) - pow(d2,2) + pow(x2,2) - pow(x4,2) + pow(y2,2) - pow(y4,2) + pow(z2,2) - pow(z4,2);

    a3 = 2*(x3-x4);
    b3 = 2*(y3-y4);
    c3 = 2*(z3-z4);
    r3 = pow(d4,2) - pow(d3,2) + pow(x3,2) - pow(x4,2) + pow(y3,2) - pow(y4,2) + pow(z3,2) - pow(z4,2);

    //化简成二元一次方程组
    double e1, f1, s1;
    double e2, f2, s2;
    e1 = a1*c2 - a2*c1;
    f1 = b1*c2 - b2*c1;
    s1 = r1*c2 - r2*c1;

    e2 = a2*c3 - a3*c2;
    f2 = b2*c3 - b3*c2;
    s2 = r2*c3 - r3*c2;

    //二元一次方程组求解
    double det = (e2*f1-e1*f2);

    if(det == 0.0)
    {
        return false;
    }

    report->x = (s2*f1-s1*f2)/(det);
    report->y = (s1*e2-s2*e1)/(det);
    report->z = (r1 - a1*report->x - b1*report->y) / c1;
    return true;
}
