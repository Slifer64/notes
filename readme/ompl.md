
## Solve with dynamic time limit

The problem to be solved is set with `ompl::base::Planner::setProblemDefinition()`. The <font size=2 style="color: #000000; background-color: #ffff88">**ompl::base::Planner::solve() method can be called repeatedly with different allowed time durations until a solution is found**</font>. The planning process continues with the available data structures when sequential calls to o`mpl::base::Planner::solve()` are made. A call to <font size=2 style="color: #000000; background-color: #ffff88">**ompl::base::Planner::clear() restores the planner's state**</font> to that before any calls to the `ompl::base::Planner::solve()` method were made.


## TODO
<font size=2 style="color: #000000; background-color: #ffff88">**ompl::base::StateValidityChecker**</font> is an abstract class that provides functionality for determining whether a state is valid or not. This class <font size=2 style="color: #000000; background-color: #ffff88">**must be thread safe**</font>. The user should provide an implementation of this class and supply it to the space information instance by calling `ompl::base::SpaceInformation::setStateValidityChecker()`. Alternatively, the user can pass a function of the type `ompl::base::StateValidityCheckerFn` to `ompl::base::SpaceInformation::setStateValidityChecker()` instead. By default, all states are considered valid, if this parameter is not set.

if using `ompl::base::DiscreteMotionValidator` for validating motions (this is the default), a call needs to be made to `ompl::base::SpaceInformation::setStateValidityCheckingResolution()` in order to specify the <font size=2 style="color: #000000; background-color: #ffff88">**maximum distance between states to be checked for validity along a path segment**</font>. This distance is specified as a percentage of a space's maximum extent. If this call is not made, <font size=2 style="color: #000000; background-color: #ffff88">**the default resolution is 1%**</font>. This value may be too low, in which case planning will be slower, or it may be too high, in which case it is possible to have collisions in solution plans.


## TODO
Notice that in ClearanceObjective's constructor we initialized the `ompl::base::StateCostIntegralObjective` with the additional argument `true`. This changes the behaviour of the objective to use motion cost interpolation when summing up state costs along the path. By default, `ompl::base::StateCostIntegralObjective` simply takes the individual states that make up a given path, and sums up those costs. However, this approach can result in an inaccurate estimation of the path cost if successive states on the path are far apart. If we enable motion cost interpolation the path cost computation will interpolate between distant states in order to get a more accurate approximation of the true path cost. This interpolation of states along a path is the same as the one used in `ompl::base::DiscreteMotionValidator`. Note that the increase in accuracy by using motion cost interpolation comes with a decrease in computational effiency due to more calls to `ompl::base::OptimizationObjective::stateCost`.
```cpp
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }
 
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};
```

## Set optimality threshold
The default behaviour for `ompl::base::PathLengthOptimizationObjective` is to set the threshold to `0.0` if a threshold wasn't specified. This means that the objective is only satisfied by paths of length less than `0.0`, which will never be satisfied, so the planner will return the best possible path within the time limit.
We can create an `OptimizationObjective` with a quality threshold of `1.51` by using the `setCostThreshold` method.
The planner will terminate when a path shorter than the given threshold is found.
```cpp
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}
```

# Optimization objective

## Specifying a new optimization objective

```cpp
class MaximizeMinClearance : public ob::OptimizationObjective
{
public:
    MaximizeMinClearance(const ob::SpaceInformationPtr &si) :
        ob::OptimizationObjective(si) {}
 
    virtual ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(this->si_->getStateValidityChecker()->clearance(s));
    }

    virtual bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const override
    {
        return c1.value() > c2.value() + ompl::magic::BETTER_PATH_COST_MARGIN;
    }

    /*
    Technically, the cost of the motion comes from the minimum clearance over the entire continuum of states along that motion. In most real-world motion planning problems this is very difficult to compute. One approximation is to take the the minimum of the clearances of the two endpoints; we'll implement this approximation as an example for simplicity, but it's a much better idea to sample some interpolating states along the motion for more accuracy, as is done in ompl::base::MinimaxObjective::motionCost
    */
    virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
    {
        return this->combineCosts(this->stateCost(s1), this->stateCost(s2));
    }

    virtual ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const override
    {
        if (c1.value() < c2.value())
            return c1;
        else
            return c2;
    }

    /*
    This is a cost value c0 which, when combined with any other cost value c1 using combineCost, always returns the value c1.
    */
    virtual ob::Cost identityCost() const override
    {
        return ob::Cost(std::numeric_limits<double>::infinity());
    }

    /*
    This is a cost which is worse than all other cost values; in other words, it's a value c_i for which isCostBetterThan(c_i, c) is false for all values of c.
    */
    virtual ob::Cost infiniteCost() const override
    {
        return ob::Cost(-std::numeric_limits<double>::infinity());
    }
};
```

## Cost heuristics

Some optimizing motion planners such as `ompl::geometric::PRMstar` can plan more efficiently if you provide them with cost heuristics.

The heuristic costs must be admissible! (otherwise optimality is compromised.)

### Motion cost heuristics

Approximate the cost of the optimal path between two given states.

Motion planners typically have greater speedups when heuristics more accurately approximate the true motion cost. Therefore, if your optimal planning problem allows for a more accurate and quick-to-compute admissible heuristic, it is recommended to provide one by implementing `ompl::base::OptimizationObjective::motionCostHeuristic`.

Example for a 2D point robot:
```cpp
ompl::base::Cost
ompl::base::PathLengthOptimizationObjective::motionCostHeuristic(const State *s1,
                                                                 const State *s2) const
{
    return Cost(si_->distance(s1, s2));
}
```

### Cost-to-go heuristics

Approximate the cost of the optimal path between a given state and the goal.

```cpp
ompl::base::Cost ompl::base::goalRegionCostToGo(const State* state, const Goal* goal)
{
    const GoalRegion* goalRegion = goal->as<GoalRegion>();
 
    // Ensures that all states within the goal region's threshold to
    // have a cost-to-go of exactly zero.
    return Cost(std::max(goalRegion->distanceGoal(state) - goalRegion->getThreshold(),
                         0.0));
}

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}
```

**Note**: `the ompl::base::goalRegionCostToGo` heuristic is only valid for your planning problem if `ompl::base::GoalRegion::distanceGoal` is an admissible heuristic on the optimal path cost from a state to your goal region. For instance, if you're planning with `ompl::base::MaximizeMinClearanceObjective` to maximize minimum path clearance, the `ompl::base::goalRegionCostToGo` function would not be a suitable cost-to-go heuristic because `ompl::base::GoalRegion::distanceGoal` has no correlation with path clearance.