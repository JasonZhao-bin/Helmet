#include "tag_pos.h"
/*
 *	���ܣ����ݻ�վ������ֵ���ж��Ƿ���Ч������
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
 *	һά��λ�㷨�����ջ�վ���껭Բ�������һ���������
 */



// ��������ͬ
int double_equals(double const a, double const b)
{
    static const double ZERO = 1e-9;
    return fabs(a - b) < ZERO;
}

// ����֮������ƽ��
double distance_sqr(struct cross_point_t const* a, struct cross_point_t const* b)
{
    return (a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y);
}

// ����֮��ľ���
double distance(struct cross_point_t const* a, struct cross_point_t const* b)
{
    return sqrt(distance_sqr(a, b));
}

/*
* ��Բ�ཻ����
* ����:
*    circles[0] �� circles[1] �ֱ�������Բ.
*    points[0] �� points[1] ������Ž�����ֵ, ��Ȼ��Щ�������������������;
*        ����õ�����������, ��ô���غ�, ����������ǰ, ���������һ��, ������������ǰ.
* ����ֵ:
*    -1 �������Բһģһ��;
*    ��������ֵ: �������.
*/
int Get_Cross_circle(struct circle_t circles[], struct cross_point_t points[])
{
    double d, a, b, c, p, q, r; // a, b, c, p, q, r ����������е���һ��
    double cos_value[2], sin_value[2]; // ������� circles[0] �϶�Ӧ��������ȡֵ
                                       // ����ֵ cos_value ���Ƿ����е� cos��
    if (double_equals(circles[0].center.x, circles[1].center.x)
        && double_equals(circles[0].center.y, circles[1].center.y)
        && double_equals(circles[0].r, circles[1].r)) {
        return CIRCLE_CROSS_INVALID;
    }

    d = distance(&circles[0].center, &circles[1].center); // Բ�ľ���
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

    // ��������һ��
    if (double_equals(d, circles[0].r + circles[1].r)
        || double_equals(d, fabs(circles[0].r - circles[1].r))) {
        cos_value[0] = -q / p / 2.0;
        sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);

        points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
        points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;

        // ��������֤���Ƿ���ȷ, �������ȷ, ����������Ž��б任
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

    // ��֤���Ƿ���ȷ, �����ⶼ��Ҫ��֤.
    if (!double_equals(distance_sqr(&points[0], &circles[1].center),
                       circles[1].r * circles[1].r)) {
        points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
    }
    if (!double_equals(distance_sqr(&points[1], &circles[1].center),
                       circles[1].r * circles[1].r)) {
        points[1].y = circles[0].center.y - circles[0].r * sin_value[1];
    }
    // �����õ�������������ͬ, ���Ȼ����һ����������귴�ſ��������һ������
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
                if(circle[0].center.x == circle[1].center.x)		//x����ͬ
                {
                    report->x = circle[0].center.x;
                    report->y = cross_point[0].y;
                    report->z = 0.0;
                }
                else if(circle[0].center.y == circle[1].center.y)	//y����ͬ
                {
                    report->x = cross_point[0].x;
                    report->y = circle[0].center.y;
                    report->z = 0.0;
                }
                else //�������
                {
                    report->x = cross_point[0].x;
                    report->y = cross_point[0].y;
                    report->z = 0.0;
                }
            break;
        case CIRCLE_CROSS_TWO:
                if(circle[0].center.x == circle[1].center.x)	//x����ͬ
                {
                    report->x = circle[0].center.x;
                    report->y = (cross_point[0].y + cross_point[1].y)/2.0;
                    report->z = 0.0;
                }
                else if(circle[0].center.y == circle[1].center.y) //y����ͬ
                {
                    report->x = (cross_point[0].x + cross_point[1].x)/2.0;
                    report->y = circle[0].center.y;
                    report->z = 0.0;
                }
                else //�������
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
 *	���ܣ����������㷨����һά����
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
        //һά�������ɹ�
        ret = true;
    }
    else
    {
        //һά�������ʧ��
    }


    return ret;
}


/*
 *	���ܣ����������㷨�����ά����(����Ԫһ�η����� ����)
 *
 */
bool Get_trilateration_2Dimen(double x1, double y1, double d1,
                              double x2, double y2, double d2,
                              double x3, double y3, double d3,
                              vec3d_t *report)
{
//	if(is_valid_Anccoord(x1, y1, x2, y2, x3, y3) == false)
//	{
//		_dbg_printf("�����ϵĻ�վ����\n");
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

    //����ĸ�Ƿ�Ϊ0,���Ϊ�����˳�
    if(det == 0.0)
    {
        return false;
    }

    report->x = (r1 * b2 - r2 * b1) / det;
    report->y = (r2 * a1 - r1 * a2) / det;

    return true;
}





/*
 *	���ܣ����������㷨������ά����(�����Ԫһ�η����� ����)
 *
 */
bool Get_trilateration_3Dimen(double x1, double y1, double z1, double d1,
                              double x2, double y2, double z2, double d2,
                              double x3, double y3, double z3, double d3,
                              double x4, double y4, double z4, double d4,
                              vec3d_t *report)
{
    //��Ԫһ�η��������
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

    //����ɶ�Ԫһ�η�����
    double e1, f1, s1;
    double e2, f2, s2;
    e1 = a1*c2 - a2*c1;
    f1 = b1*c2 - b2*c1;
    s1 = r1*c2 - r2*c1;

    e2 = a2*c3 - a3*c2;
    f2 = b2*c3 - b3*c2;
    s2 = r2*c3 - r3*c2;

    //��Ԫһ�η��������
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
