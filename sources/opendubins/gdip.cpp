/*
 * gdip.cpp - Generalized Dubins Interval Problem - shortest path connecting tow states with heading intervals
 *
 *  Created on: Mar 8, 2016
 *      Author: Petr Vana
 */

#include <iomanip>
#include "dubins.h"
#include "circle.h"
//#include "test/deadcodetest.h"

using namespace std;

namespace opendubins {

    Dubins::Dubins(AngleInterval from, AngleInterval to, double newRadius, double diff1, double diff2) {
        radius = newRadius;
        length = std::numeric_limits<double>::max();
        type = DType::Unknown;
        initGDIP(from, to, diff1, diff2);
    }

    Dubins::Dubins(AngleInterval from, AngleInterval to, double newRadius, double diff) {
        radius = newRadius;
        length = std::numeric_limits<double>::max();
        type = DType::Unknown;
        initGDIP(from, to, diff);
    }

    void Dubins::GDIP_S(const AngleInterval &from, const AngleInterval &to, const double &diff2) {
        // ends of intervals:
        // from(A,B)
        const auto A = from.getRight();
        const auto B = from.getLeft();
        //  to(C,D)
        const auto C = to.getRight();
        const auto D = to.getLeft();

        // intersection of intervals from(A,B) to (C,D)
        // detection is based on  1) is A in (C,D)?    2) is B in (C,D)?
        bool AinCD = to.inInterval(A);
        bool BinCD = to.inInterval(B);

        int numberOfIntersection = 0;
        AngleInterval firstInterval;
        AngleInterval secondInterval;

        // start is consicered as a point and end as a circle with radius = diff2
        firstInterval.point = secondInterval.point = from.point;

        if (AinCD && BinCD) {
            // there are two options: (A,B) or {(C,B),(A,D)}
            if (angleToLeft(C, B) > angleToLeft(C, A)) {
                // intersection is (A,B)
                numberOfIntersection = 1;
                firstInterval = from;
            } else {
                // intersection is {(C,B),(A,D)}
                numberOfIntersection = 2;

                firstInterval.rightDir = C;
                firstInterval.diff = angleToLeft(C, B);

                secondInterval.rightDir = A;
                secondInterval.diff = angleToLeft(A, D);
            }
        } else if (AinCD) { // not BinCD
            // intersection is (A,D)
            numberOfIntersection = 1;
            firstInterval.rightDir = A;
            firstInterval.diff = angleToLeft(A, D);
        } else if (BinCD) { // not AinCD
            // intersection is (C,B)
            numberOfIntersection = 1;
            firstInterval.rightDir = C;
            firstInterval.diff = angleToLeft(C, B);
        } else {
            if (from.inInterval(C) || from.inInterval(D)) {
                // intersection is (C,D)
                numberOfIntersection = 1;
                firstInterval.rightDir = C;
                firstInterval.diff = angleToLeft(C, D);
            }
        }

        if (numberOfIntersection > 0) { // first interval
            GDIP_S(firstInterval, to.point, diff2);
        }

        if (numberOfIntersection > 1) { // second interval
            GDIP_S(secondInterval, to.point, diff2);
        }
    }

    void Dubins::GDIP_S(const AngleInterval &from, const Point &to, const double &diff2) {
        Vector diff = to - from.point;
        double diffAng = diff.getAngle();

        if (diff.length() < diff2) { // no move needed
            isCCC = false;
            length = len1 = len2 = len3 = 0;
            double ang = from.rightDir + from.diff / 2;
            start = State(from.point, ang);
            end = State(from.point, ang);
            type = DType::GDIP_NO;
            //USED(GDIP, 1);
        } else {
            if (from.inInterval(diffAng)) { // the shortest S maneuver is in interval
                auto intersection = to + diff.normalize() * -diff2;
                auto nLen = intersection.distance(from.point);
                if (nLen < length) {
                    isCCC = false;
                    len1 = len3 = 0;
                    length = len2 = nLen;
                    start = State(from.point, diffAng);
                    end = State(intersection, diffAng);
                    type = DType::GDIP_S;
                    //USED(GDIP, 2);
                }
            } else { // solution is at the edge of the interval
                GDIP_S_onEdge(from, to, diff2, from.getRight());
                GDIP_S_onEdge(from, to, diff2, from.getLeft());
            } // else there is no intersection with the neighborhood circle
        }
    }

    // todo - remove because it is not necessary
    void Dubins::GDIP_S_onEdge(const AngleInterval &from, const Point &to, const double &diff2, double dir) {
        Circle ng = Circle(to, diff2);

        State halfLine1 = State(from.point, dir);
        auto intersection = ng.halfLineIntersection(halfLine1);
        if (intersection.isValid()) {
            auto nLen = from.point.distance(intersection);
            if (nLen < length) {
                isCCC = false;
                len1 = len3 = 0;
                length = len2 = nLen;
                start = State(from.point, halfLine1.ang);
                end = State(intersection, halfLine1.ang);
                type = DType::GDIP_S;
                //USED(GDIP, 3);
            }
        }
    }

    void Dubins::GDIP_CS(const State &from, const AngleInterval &to, const double &diff2, bool isLeft) {

        const auto totalDiff = diff2;

        Vector dir1radius = from.getNormalizedDirection() * radius;
        // center of the turning circle
        Point c1right = from.point + dir1radius.right();
        Point c1left = from.point + dir1radius.left();

        // vector from the center of the right turn to the goal point
        Vector diff = to.point - (isLeft ? c1left : c1right);

        double len = diff.length();
        if (radius <= len) { // RS maneuver
            double alpha = asin(radius / len);
            double directionS = diff.getAngle() + (isLeft ? alpha : -alpha);
            double turn = isLeft ? angleToLeft(from.ang, directionS) : angleToRight(from.ang, directionS);

            State st = from;
            Arc arc(st, turn, radius);
            double lenS = arc.getEnd().point.distance(to.point);

            // todo - to be tested
            if (to.inInterval(directionS)) {
                // end of R segments leads directly to center of the goal area (to.point)
                auto lenS2 = max(0.0, lenS - totalDiff);
                //auto lenS2 = lenS;
                double nLen = lenS2 + radius * fabs(turn);
                if (nLen < length) {
                    length = nLen;
                    isCCC = false;
                    len1 = turn;
                    len2 = lenS2;
                    len3 = 0;
                    start = st;
                    end = State(to.point - Vector(directionS) * (lenS - lenS2), directionS);
                    type = isLeft ? DType::GDIP_LS : DType::GDIP_RS;
                    //USED(GDIP, 4);
                }
            }
        }

        {
            // termination angle is on right edge of the termination interval
            GDIP_CS_onEdge(from, to, diff2, to.getLeft(), isLeft);

            // termination angle is on left edge of the termination interval
            GDIP_CS_onEdge(from, to, diff2, to.getRight(), isLeft);
        }

        { // possible C maneuver
            Circle goal(to.point, diff2);

            { // C maneuver ends when intersecting goal area (circle)
                Arc arc(from, (isLeft ? 2 : -2) * M_PI, radius);

                State interState;
                double interDist;
                tie(interState, interDist) = goal.firstIntersection(arc);

                if (interState.point.isValid()) {
                    auto nLen = interDist;
                    if (to.inInterval(interState.ang)) {
                        if (nLen < length) {
                            length = nLen;
                            isCCC = false;
                            len1 = (isLeft ? 1 : -1) * nLen / radius;
                            len2 = 0;
                            len3 = 0;
                            start = from;
                            end = interState;
                            type = isLeft ? DType::GDIP_L : DType::GDIP_R;
                            //USED(GDIP, 5);
                        }
                    }
                }
            }

            { // C maneuver ends when the terminal heading interval is met
                double terminalAngle = isLeft ? to.getRight() : to.getLeft();
                double turn = isLeft ? angleToLeft(from.ang, terminalAngle) : angleToRight(from.ang, terminalAngle);

                double nLen = radius * fabs(turn);
                if (nLen < length) {
                    Arc arc(from, turn, radius);
                    auto newEnd = arc.getEnd();

                    if (goal.pointInCircle(newEnd.point)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = 0;
                        len3 = 0;
                        start = from;
                        end = newEnd;
                        type = isLeft ? DType::GDIP_L : DType::GDIP_R;
                        //USED(GDIP, 6);
                    }

                }
            }
        }
    }

    void Dubins::GDIP_CS_onEdge(const State &from, const AngleInterval &to, const double &diff2, double dir, const bool& isLeft) {
        Circle goal(to.point, diff2);

        auto turn2 = isLeft ? angleToLeft(from.ang, dir) : angleToRight(from.ang, dir);
        auto rEnd2 = Arc(from, turn2, radius).getEnd();

        auto intersect2 = goal.halfLineIntersection(rEnd2);

        if (intersect2.isValid()) {
            auto lenS = intersect2.distance(rEnd2.point);
            auto nLen = lenS + radius * fabs(turn2);
            if (nLen < length) {
                length = nLen;
                isCCC = false;
                len1 = turn2;
                len2 = lenS;
                len3 = 0;
                start = from;
                end = State(intersect2, rEnd2.ang);
                type = isLeft ? DType::GDIP_LS : DType::GDIP_RS;
                //USED(GDIP, 4);
            }
        }
    }

    void Dubins::GDIP_SC(const AngleInterval& from, const AngleInterval& to, const double& diff2) {
        Dubins act;
        act.radius = radius;

        AngleInterval revertedFrom = from.reverse();
        AngleInterval revertedTo = to.reverse();

        // reverted S and SL maneuver ... to > from
        act.GDIP_CS(revertedTo.getLeftState(), revertedFrom, diff2, true);

        // reverted S and SR maneuver ... to > from
        act.GDIP_CS(revertedTo.getRightState(), revertedFrom, diff2, false);

        if(act.length < length){
            length = act.length;
            isCCC = act.isCCC;
            len1 = -act.len3;
            len2 = act.len2;
            len3 = -act.len1;
            // todo - move this points in states
            start = act.end.reverse();
            end = act.start.reverse();
            type = DType::Unknown;
            if(act.type == DType::GDIP_L){
                type = DType::GDIP_R;
            }else if(act.type == DType::GDIP_R) {
                type = DType::GDIP_L;
            }else if(act.type == DType::GDIP_RS){
                type = DType::GDIP_SL;
            }else if(act.type == DType::GDIP_LS){
                type = DType::GDIP_SR;
            }
        }
    }

    void Dubins::GDIP_CSC_same_side(const State &from, const State &to, const double &diff2, const bool &isLeft) {
        double totalTurn = angleToSide(from.ang, to.ang, isLeft);

        Point target = Arc(from, totalTurn, radius).getEnd().point;
        double rightDir;
        double diff;
        if(isLeft){
            rightDir = from.ang;
            diff = totalTurn;
        }else{
            rightDir = to.ang;
            diff = -totalTurn;
        }

        AngleInterval targetInterval = AngleInterval(target, rightDir, diff);
        Circle goal(to.point, diff2);

        // find shortest point in the goal area from target
        // direction from terget needs to be in targetInterval
        {
            Vector dir = goal.shortestVectorInSector(targetInterval);
            double dirAng = dir.getAngle();

            double turn1 = angleToSide(from.ang, dirAng, isLeft);
            double straight = dir.length();
            double turn2 = angleToSide(dirAng, to.ang, isLeft);

            double nLen = radius * (fabs(turn1) + fabs(turn2)) + straight;

            if (nLen < length) {
                length = nLen;
                isCCC = false;
                len1 = turn1;
                len2 = straight;
                len3 = turn2;
                type = isLeft ? DType::GDIP_LSL : DType::GDIP_RSR;
                start = from;
                end = getSecondArc().getEnd();
            }
        }

        // full turn
        {
            Vector dir = goal.center - targetInterval.point;
            double dirAng = dir.getAngle();

            double turn1 = angleToSide(from.ang, dirAng, isLeft);
            double straight = max(0.0, dir.length() - goal.radius);
            double turn2 = angleToSide(dirAng, to.ang, isLeft);

            double nLen = radius * (fabs(turn1) + fabs(turn2)) + straight;

            if (nLen < length) {
                length = nLen;
                isCCC = false;
                len1 = turn1;
                len2 = straight;
                len3 = turn2;
                type = isLeft ? DType::GDIP_LSL : DType::GDIP_RSR;
                start = from;
                end = getSecondArc().getEnd();
            }
        }
    }

    void Dubins::Dubins_StateToCircle(const State& from, const Circle& goal, const double& radius, const bool& isLeft,
                              double& turn, double& straight, double& length) {

        double sgn = isLeft ? 1 : -1;

        turn = NAN;
        straight = NAN;
        length = numeric_limits<double>::max();

        // no maneuver
        if(goal.pointInCircle(from.point)){
            turn = 0;
            straight = 0;
            length = 0;
            return;
        }

        Arc arc(from, sgn, radius);

        // C case - intersection of the goal and the arc

        State interState;
        double interDist;
        tie(interState, interDist) = goal.firstIntersection(arc);

        if(!std::isnan(interState.point.x)){
            turn = interDist / radius * sgn;
            straight = 0;
            length = interDist;
            // todo - remove
            //cout << interState.point.distance(goal.center) << " = " << goal.radius << endl;
            //cout << arc.getState(interDist).point.distance(goal.center) << " = " << goal.radius << endl;
        }

        // CS case - to the goal center
        Vector dir1radius = from.getNormalizedDirection() * radius;
        // center of the turning circle
        Point c1right = from.point + dir1radius.right();
        Point c1left = from.point + dir1radius.left();

        // vector from the center of the right turn to the goal point
        Vector diff = goal.center - (isLeft ? c1left : c1right);

        double len = diff.length();
        if (radius <= len) { // RS maneuver
            double alpha = asin(radius / len);
            double directionS = diff.getAngle() + (isLeft ? alpha : -alpha);
            double nTurn = angleToSide(from.ang, directionS, isLeft);

            State st = from;
            Arc arc(st, nTurn, radius);
            double lenS = arc.getEnd().point.distance(goal.center);

            // end of R segments leads directly to center of the goal area (to.point)
            auto lenS2 = max(0.0, lenS - goal.radius);
            double nLen = lenS2 + radius * fabs(nTurn);
            if (nLen < length) {
                length = nLen;
                turn = nTurn;
                straight = lenS2;
            }
        }

    }

    void Dubins::GDIP_CSC_diff_side(const State &from, const State &to, const double &diff2, const bool &isLeft) {
        // the first turn is longer
        {
            double turn1 = angleToSide(from.ang, to.ang, isLeft);
            State from2 = Arc(from, turn1, radius).getEnd();

            Point halfGoalPoint = from2.point + ((to.point - from2.point) * 0.5);
            Circle halfGoal(halfGoalPoint, diff2 / 2);

            double turnSTC, straightSTC, lengthSTC;
            Dubins_StateToCircle(from2, halfGoal, radius, isLeft, turnSTC, straightSTC, lengthSTC);

            double nLen = radius*(fabs(turn1) + 2*fabs(turnSTC)) + 2*straightSTC;
            if(nLen < length){
                length = nLen;
                len1 = turn1 + turnSTC;
                len2 = straightSTC * 2;
                len3 = -turnSTC;
                isCCC = false;
                start = from;
                end = getSecondArc().getEnd();
                type = isLeft ? DType::GDIP_LSR : DType::GDIP_RSL;
            }
        }

        // the second turn is longer
        {
            double turn2 = angleToSide(from.ang, to.ang, !isLeft);
            State to2 = Arc(to.reverse(), -turn2, radius).getEnd().reverse();

            Point halfGoalPoint = from.point + ((to2.point - from.point) * 0.5);
            Circle halfGoal(halfGoalPoint, diff2 / 2);

            double turnSTC, straightSTC, lengthSTC;
            Dubins_StateToCircle(from, halfGoal, radius, isLeft, turnSTC, straightSTC, lengthSTC);

            double nLen = radius*(fabs(turn2) + 2*fabs(turnSTC)) + 2*straightSTC;
            if(nLen < length){
                length = nLen;
                len1 = turnSTC;
                len2 = straightSTC * 2;
                len3 = -turnSTC + turn2;
                isCCC = false;
                start = from;
                end = getSecondArc().getEnd();
                type = isLeft ? DType::GDIP_LSR : DType::GDIP_RSL;
            }
        }

    }

    void Dubins::initGDIP(AngleInterval from, AngleInterval to, double diff2) {

        initIntervalProblem(from, to);

        // S maneuver
        GDIP_S(from, to, diff2);

        // L and LS maneuver
        GDIP_CS(from.getLeftState(), to, diff2, true);

        // R and RS maneuver
        GDIP_CS(from.getRightState(), to, diff2, false);

        // SL nad SR maneuvers
        GDIP_SC(from, to, diff2);

        // LSL - same side
        GDIP_CSC_same_side(from.getLeftState(), to.getRightState(), diff2, true);

        // RSR - same side
        GDIP_CSC_same_side(from.getRightState(), to.getLeftState(), diff2, false);

        // LSR - same side
        GDIP_CSC_diff_side(from.getLeftState(), to.getLeftState(), diff2, true);

        // RSL - same side
        GDIP_CSC_diff_side(from.getRightState(), to.getRightState(), diff2, false);


        // TODO - implement other types

    }

    void Dubins::initGDIP(AngleInterval from, AngleInterval to, double diff1, double diff2) {

        initGDIP(from, to, diff1 + diff2);

        // change positions of start and end to meet distance conditions (diff1 and diff2)
        auto diffVector1 = start.point - from.point;
        auto diffVector2 = end.point - to.point;

        auto diff = diff1 + diff2;
        if (diff > 0) {
            auto alpha1 = diff1 / diff;
            auto alpha2 = 1 - alpha1;

            start.point = from.point + alpha1 * diffVector1 - alpha1 * diffVector2;
            end.point = to.point + alpha2 * diffVector2 - alpha2 * diffVector1;
        }
    }

}
