#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <random>

#ifndef SIMPLE_RRT_PLANNER_HPP
#define SIMPLE_RRT_PLANNER_HPP

namespace simple_rrt_planner
{
    template<typename T, typename Allocator=std::allocator<T>>
    class SimpleRRTPlannerState
    {
    protected:

        bool initialized_;
        int64_t parent_index_;
        std::vector<int64_t> child_indices_;
        T value_;

    public:

        SimpleRRTPlannerState() : initialized_(false), parent_index_(-1)
        {
            child_indices_.clear();
        }

        SimpleRRTPlannerState(const T& value, const int64_t parent_index, const std::vector<int64_t>& child_indices)
        {
            parent_index_ = parent_index;
            child_indices_ = child_indices;
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerState(const T& value, const int64_t parent_index)
        {
            parent_index_ = parent_index;
            child_indices_.clear();
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerState(const T& value)
        {
            parent_index_ = -1;
            child_indices_.clear();
            value_ = value;
            initialized_ = true;
        }

        bool IsInitialized() const
        {
            return initialized_;
        }

        const T& GetValueImmutable() const
        {
            return value_;
        }

        T& GetValueMutable()
        {
            return value_;
        }

        int64_t GetParentIndex() const
        {
            return parent_index_;
        }

        void SetParentIndex(const int64_t parent_index)
        {
            parent_index_ = parent_index;
        }

        const std::vector<int64_t>& GetChildIndices() const
        {
            return child_indices_;
        }

        void ClearChildIndicies()
        {
            child_indices_.clear();
        }

        void AddChildIndex(const int64_t child_index)
        {
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] == child_index)
                {
                    return;
                }
            }
            child_indices_.push_back(child_index);
        }

        void RemoveChildIndex(const int64_t child_index)
        {
            std::vector<int64_t> new_child_indices;
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] != child_index)
                {
                    new_child_indices.push_back(child_indices_[idx]);
                }
            }
            child_indices_ = new_child_indices;
        }
    };

    template<typename T, typename Allocator=std::allocator<T>>
    class SimpleRRTPlannerPointerState
    {
    protected:

        bool initialized_;
        std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>> parent_;
        T value_;

    public:

        SimpleRRTPlannerPointerState() : initialized_(false)
        {
            parent_ = std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>();
        }

        SimpleRRTPlannerPointerState(const T& value, const std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>& parent)
        {
            parent_(parent);
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerPointerState(const T& value)
        {
            parent_ = std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>();
            value_ = value;
            initialized_ = true;
        }

        bool IsInitialized() const
        {
            return initialized_;
        }

        const T& GetValueImmutable() const
        {
            return value_;
        }

        T& GetValueMutable()
        {
            return value_;
        }

        const std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>& GetParent() const
        {
            return parent_;
        }

        void SetParent(const std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>& parent)
        {
            parent_(parent);
        }
    };



    template<typename T, typename Allocator=std::allocator<T>>
    class SimpleHybridRRTPlanner
    {
    private:

        SimpleHybridRRTPlanner() {}


    public:

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * time_limit - limit, in seconds, for the runtime of the planner
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        static std::pair<std::vector<T>, std::map<std::string, double>> Plan(const T& start,
                                                                      const T& goal,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&, const T&)>& goal_reached_fn,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                                                                      const std::chrono::duration<double>& time_limit)
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            std::function<bool(void)> termination_check_fn = [&](void) { return (((std::chrono::time_point<std::chrono::high_resolution_clock>)std::chrono::high_resolution_clock::now() - start_time) > time_limit); };
            // Define a couple lambdas to let us use the generic multi-path planner as if it were a single-path planner
            bool solution_found = false;
            std::function<bool(const T&, const T&)> real_goal_found_fn = [&](const T& state, const T& goal) { if (goal_reached_fn(state,goal)) { solution_found = true; return true; } else {return false;} };
            std::function<bool(void)> real_termination_check_fn = [&](void) { if (!solution_found) { return termination_check_fn(); } else {return true;} };
            std::function<void(SimpleRRTPlannerState<T, Allocator>&)> dummy_goal_callback_fn = [](SimpleRRTPlannerState<T, Allocator>& state) {;};
            // Call the planner
            std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> planning_result = PlanMultiPath(start, goal, nearest_neighbor_fn, real_goal_found_fn, dummy_goal_callback_fn, sampling_fn, forward_propagation_fn, real_termination_check_fn);
            // Put together the return
            std::vector<T> planned_path;
            if (planning_result.first.size() > 0)
            {
                planned_path = planning_result.first[0];
            }
            return std::pair<std::vector<T>, std::map<std::string, double>>(planned_path, planning_result.second);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<paths, statistics>
         * paths - vector of vector of states corresponding to the planned path(s)
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        static std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> PlanMultiPath(const T& start,
                                                                      const T& goal,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&, const T&)>& goal_reached_fn,
                                                                      std::function<void(SimpleRRTPlannerState<T, Allocator>&)>& goal_reached_callback_fn,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                                                                      std::function<bool(void)>& termination_check_fn)
        {
            // Keep track of states
            std::vector<SimpleRRTPlannerState<T, Allocator>> nodes;
            nodes.clear();
            // Add the start state
            SimpleRRTPlannerState<T, Allocator> start_state(start);
            nodes.push_back(start_state);

            //dummy state added function
            std::function<void(SimpleRRTPlannerState<T, Allocator>&, SimpleRRTPlannerState<T, Allocator>&)> state_added_fn = [] (SimpleRRTPlannerState<T, Allocator>& parent, SimpleRRTPlannerState<T, Allocator>& new_child) { ; };
            
            // Make sure we've been given a start state
            assert(nodes.size() > 0);
            // Make sure the tree is properly linked
            assert(CheckTreeLinkage(nodes));
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            // Storage for the goal states we reach
            std::vector<int64_t> goal_state_indices;
            // Safety check before doing real work
            if (goal_reached_fn(nodes[0].GetValueImmutable(),goal))
            {
                goal_state_indices.push_back(0);
                std::cerr << "Start state meets goal conditions, returning default path [start]" << std::endl;
                // Put together the results
                std::vector<std::vector<T>> planned_paths = ExtractSolutionPaths(nodes, goal_state_indices);
                statistics["planning_time"] = 0.0;
                statistics["total_states"] = nodes.size();
                statistics["solutions"] = (double)planned_paths.size();
                return std::pair<std::vector<std::vector<T>>, std::map<std::string, double>>(planned_paths, statistics);
            }
            // Update the start time
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            // Plan
            while (!termination_check_fn())
            {
                // Sample a random goal
                T random_target = sampling_fn();
                // Get the nearest neighbor
                int64_t nearest_neighbor_index = nearest_neighbor_fn(nodes, random_target);
                assert((nearest_neighbor_index >= 0) && (nearest_neighbor_index < nodes.size()));
                const T& nearest_neighbor = nodes.at(nearest_neighbor_index).GetValueImmutable();
                // Forward propagate towards the goal
                std::vector<std::pair<T, int64_t>> propagated = forward_propagation_fn(nearest_neighbor, random_target);
                if (!propagated.empty())
                {
                    statistics["total_samples"] += 1.0;
                    statistics["successful_samples"] += 1.0;
                    for (size_t idx = 0; idx < propagated.size(); idx++)
                    {
                        const std::pair<T, int64_t>& current_propagation = propagated[idx];
                        // Determine the parent index of the new state
                        // This process deserves some explanation
                        // The "current relative parent index" is the index of the parent, relative to the list of propagated nodes.
                        // A negative value means the nearest neighbor in the tree, zero means the first propagated node, and so on.
                        // NOTE - the relative parent index *must* be lower than the index in the list of prograted nodes
                        // i.e. the first node must have a negative value, and so on.
                        const int64_t& current_relative_parent_index = current_propagation.second;
                        int64_t node_parent_index = nearest_neighbor_index;
                        if (current_relative_parent_index >= 0)
                        {
                            const int64_t current_relative_index = (int64_t)idx;
                            assert(current_relative_parent_index < current_relative_index);
                            const int64_t current_relative_offset = current_relative_parent_index - current_relative_index;
                            assert(current_relative_offset < 0);
                            assert(current_relative_offset >= -(int64_t)propagated.size());
                            const int64_t current_nodes_size = (int64_t)nodes.size();
                            node_parent_index = current_nodes_size + current_relative_offset; // Offset is negative!
                        }
                        else
                        {
                            node_parent_index = nearest_neighbor_index; // Negative relative parent index means our parent index is the nearest neighbor index
                        }
                        // Build the new state
                        const T& current_propagated = current_propagation.first;
                        SimpleRRTPlannerState<T, Allocator> new_state(current_propagated, node_parent_index);
                        // Add the state to the tree
                        nodes.push_back(new_state);
                        int64_t new_node_index = (int64_t)nodes.size() - 1;
                        nodes[node_parent_index].AddChildIndex(new_node_index);
                        // Call the state added callback
                        state_added_fn(nodes[node_parent_index], nodes[new_node_index]);
                        // Check if we've reached the goal
                        if (goal_reached_fn(nodes[new_node_index].GetValueImmutable(),goal))
                        {
                            goal_state_indices.push_back(new_node_index);
                            goal_reached_callback_fn(nodes[new_node_index]);
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
            }
            // Put together the results
            // Make sure the tree is properly linked
            assert(CheckTreeLinkage(nodes));
            std::vector<std::vector<T>> planned_paths = ExtractSolutionPaths(nodes, goal_state_indices);
            std::chrono::time_point<std::chrono::high_resolution_clock> cur_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> planning_time(cur_time - start_time);
            statistics["planning_time"] = planning_time.count();
            statistics["total_states"] = nodes.size();
            statistics["solutions"] = (double)planned_paths.size();
            return std::pair<std::vector<std::vector<T>>, std::map<std::string, double>>(planned_paths, statistics);
        }

        /* Checks the planner tree to make sure the parent-child linkages are correct
         */
        static bool CheckTreeLinkage(const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes)
        {
            // Step through each state in the tree. Make sure that the linkage to the parent and child states are correct
            for (size_t current_index = 0; current_index < nodes.size(); current_index++)
            {
                // For every state, make sure all the parent<->child linkages are valid
                const SimpleRRTPlannerState<T, Allocator>& current_state = nodes[current_index];
                if (!current_state.IsInitialized())
                {
                    std::cerr << "Tree contains uninitialized node(s) " << current_index << std::endl;
                    return false;
                }
                // Check the linkage to the parent state
                const int64_t parent_index = current_state.GetParentIndex();
                if ((parent_index >= 0) && (parent_index < (int64_t)nodes.size()))
                {
                    if (parent_index != (int64_t)current_index)
                    {
                        const SimpleRRTPlannerState<T, Allocator>& parent_state = nodes[parent_index];
                        if (!parent_state.IsInitialized())
                        {
                            std::cerr << "Tree contains uninitialized node(s) " << parent_index << std::endl;
                            return false;
                        }
                        // Make sure the corresponding parent contains the current node in the list of child indices
                        const std::vector<int64_t>& parent_child_indices = parent_state.GetChildIndices();
                        auto index_found = std::find(parent_child_indices.begin(), parent_child_indices.end(), (int64_t)current_index);
                        if (index_found == parent_child_indices.end())
                        {
                            std::cerr << "Parent state " << parent_index << " does not contain child index for current node " << current_index << std::endl;
                            return false;
                        }
                    }
                    else
                    {
                        std::cerr << "Invalid parent index " << parent_index << " for state " << current_index << " [Indices can't be the same]" << std::endl;
                        return false;
                    }
                }
                else if (parent_index < -1)
                {
                    std::cerr << "Invalid parent index " << parent_index << " for state " << current_index << std::endl;
                    return false;
                }
                // Check the linkage to the child states
                const std::vector<int64_t>& current_child_indices = current_state.GetChildIndices();
                for (size_t idx = 0; idx < current_child_indices.size(); idx++)
                {
                    // Get the current child index
                    const int64_t current_child_index = current_child_indices[idx];
                    if ((current_child_index > 0) && (current_child_index < (int64_t)nodes.size()))
                    {
                        if (current_child_index != (int64_t)current_index)
                        {
                            const SimpleRRTPlannerState<T, Allocator>& child_state = nodes[current_child_index];
                            if (!child_state.IsInitialized())
                            {
                                std::cerr << "Tree contains uninitialized node(s) " << current_child_index << std::endl;
                                return false;
                            }
                            // Make sure the child node points to us as the parent index
                            const int64_t child_parent_index = child_state.GetParentIndex();
                            if (child_parent_index != (int64_t)current_index)
                            {
                                std::cerr << "Parent index " << child_parent_index << " for current child state " << current_child_index << " does not match index " << current_index << " for current node " << std::endl;
                                return false;
                            }
                        }
                        else
                        {
                            std::cerr << "Invalid child index " << current_child_index << " for state " << current_index << " [Indices can't be the same]" << std::endl;
                            return false;
                        }
                    }
                    else
                    {
                        std::cerr << "Invalid child index " << current_child_index << " for state " << current_index << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }

        /* Extracts all the solution paths corresponding to the provided goal states
         */
        static std::vector<std::vector<T>> ExtractSolutionPaths(const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes, const std::vector<int64_t>& goal_state_indices)
        {
            std::vector<std::vector<T>> solution_paths;
            for (size_t idx = 0; idx < goal_state_indices.size(); idx++)
            {
                std::vector<T> solution_path = ExtractSolutionPath(nodes, goal_state_indices[idx]);
                solution_paths.push_back(solution_path);
            }
            return solution_paths;
        }

        /* Extracts a single solution path corresponding to the provided goal state
         */
        static std::vector<T> ExtractSolutionPath(const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes, const int64_t goal_state_index)
        {
            std::vector<T> solution_path;
            const SimpleRRTPlannerState<T, Allocator>& goal_state = nodes[goal_state_index];
            solution_path.push_back(goal_state.GetValueImmutable());
            int64_t parent_index = goal_state.GetParentIndex();
            while (parent_index >= 0)
            {
                assert(parent_index < nodes.size());
                const SimpleRRTPlannerState<T, Allocator>& parent_state = nodes[parent_index];
                const T& parent = parent_state.GetValueImmutable();
                solution_path.push_back(parent);
                parent_index = parent_state.GetParentIndex();
            }
            // Put it in the right order
            std::reverse(solution_path.begin(), solution_path.end());
            return solution_path;
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * time_limit - limit, in seconds, for the runtime of the planner
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        static std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> PlanMultiPath(const T& start,
                                                                      const T& goal,
                                                                      std::function<void(const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>&)>& register_nearest_neighbors_fn,
                                                                      std::function<const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>&(const T&)>& get_nearest_neighbor_fn,
                                                                      std::function<std::vector<std::vector<T>>(void)>& extract_solution_paths,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<bool(const T&, const T&)>& goal_reached_fn,
                                                                      std::function<void(const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>&)>& register_goal_state_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      std::function<bool(void)>& termination_check_fn)
        {
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_states"] = 0.0;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            // Add the start state
            SimpleRRTPlannerState<T, Allocator> start_state(start);
            register_nearest_neighbors_fn(start_state);
            // Update the start time
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            // Plan
            while (!termination_check_fn())
            {
                // Sample a random goal
                T random_target = sampling_fn();
                // Get the nearest neighbor
                const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>& nearest_neighbor_ptr = get_nearest_neighbor_fn(random_target);
                assert(nearest_neighbor_ptr);
                const T& nearest_neighbor_value = nearest_neighbor_ptr->GetValueImmutable();
                // Forward propagate towards the goal
                std::vector<T> propagated = forward_propagation_fn(nearest_neighbor_value, random_target);
                if (!propagated.empty())
                {
                    statistics["total_samples"] += 1.0;
                    statistics["successful_samples"] += 1.0;
                    std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>> parent_ptr(nearest_neighbor_ptr);
                    for (size_t idx = 0; idx < propagated.size(); idx++)
                    {
                        statistics["total_states"] += 1.0;
                        const T& current_propagated = propagated[idx];
                        std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>> new_state_ptr(new SimpleRRTPlannerPointerState<T, Allocator>(current_propagated, parent_ptr));
                        // If we've reached a goal, register it specially
                        if (goal_reached_fn(current_propagated,goal))
                        {
                            register_goal_state_fn(new_state_ptr);
                            break;
                        }
                        // Otherwise, simply register it as a nearest neighbor
                        else
                        {
                            register_nearest_neighbors_fn(new_state_ptr);
                            parent_ptr = new_state_ptr;
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
            }
            // Put together the results
            std::chrono::time_point<std::chrono::high_resolution_clock> cur_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> planning_time(cur_time - start_time);
            std::vector<std::vector<T>> planned_paths = extract_solution_paths();
            statistics["planning_time"] = planning_time.count();
            statistics["solutions"] = (double)planned_paths.size();
            return std::pair<std::vector<std::vector<T>>, std::map<std::string, double>>(planned_paths, statistics);
        }
    };

}

#endif // SIMPLE_RRT_PLANNER
