#include "fdp_boolean.h"

#include <vector>

#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

Result
FdpNotOperator::fixed_bits_both_way() {
    Result result = Result::UNCHANGED;
    auto child = d_children.at(0);
    for (size_t i = 0, isz = d_self->width(); i < isz; ++i) {
        if (d_self->is_fixed(i))
            result |= child->set_fixed(i, !d_self->get_value(i));
        else if (child->is_fixed(i))
            result |= d_self->set_fixed(i, !child->get_value(i));
        if (is_conflict(result))
            return Result::CONFLICT;
    }
    return result;
}

Result
FdpNotOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    // [min, max] => [~max, ~min] and keep complementary flag
    result |= d_self->apply_interval_constraint(
        d_children.at(0)->interval_max().bvnot(),
        d_children.at(0)->interval_min().bvnot(),
        d_children.at(0)->is_interval_complementary());

    result |= d_children.at(0)->apply_interval_constraint(
        d_self->interval_max().bvnot(),
        d_self->interval_min().bvnot(),
        d_self->is_interval_complementary());

    return result;
}

Result FdpNotOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_not);
    ++d_stats.d_num_fdp_not;

    Result result = Result::UNCHANGED;
    result |= fixed_bits_both_way();
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }
    result |= interval_both_way();
    return result;
}

std::vector<uint32_t> FdpNotOperator::implied_by() {
    return {0};
}

Result
FdpXorOperator::fixed_bits_both_way() {
    Result result = Result::UNCHANGED;

    for (size_t i = 0, isz = d_self->width(); i < isz; ++i) {
        bool one = d_self->is_fixed(i) ? d_self->get_value(i) : false;
        bool suitable = !d_self->is_fixed(i);
        size_t index = d_children.size();
        for (size_t j = 0, jsz = d_children.size(); j < jsz; ++j) {
            if (d_children.at(j)->is_fixed(i))
                one ^= (d_children.at(j)->get_value(i));
            else {
                if (suitable) {
                    suitable = false;
                    break;
                }
                suitable = true;
                index = j;
            }
        }

        if (suitable) {
            // single unknown
            if (index == d_children.size())
                result |= d_self->set_fixed(i, one);
            else
                result |= d_children.at(index)->set_fixed(i, one);
        }
    }

    return result;
}

Result
FdpXorOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    // TBD
    return result;
}

Result FdpXorOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_xor);
    ++d_stats.d_num_fdp_xor;

    Result result = Result::UNCHANGED;
    result |= fixed_bits_both_way();
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }
    result |= interval_both_way();
    return result;
}

std::vector<uint32_t> FdpXorOperator::implied_by() {
    if (d_children.at(0)->is_totally_fixed() && d_children.at(1)->is_totally_fixed())
        return {0, 1};
    else
        return {};
}

Result
FdpAndOperator::fixed_bits_both_way() {
    Result result = Result::UNCHANGED;
    for (size_t i = 0, isz = d_self->width(); i < isz; ++i) {
        if (d_self->is_fixed(i)) {  // UP -> DOWN
            if (d_self->get_value(i)) {
                for (auto child : d_children) {
                    result |= child->set_fixed(i, true);
                }
            }
            else {
                bool has_zero_child = false;
                size_t unknown_index = d_children.size();
                bool multiple_unknown = false;

                for (size_t j = 0, jsz = d_children.size(); j < jsz; ++j) {
                    if (d_children.at(j)->is_fixed(i)) {
                        if (!d_children.at(j)->get_value(i)) {
                            has_zero_child = true;
                            break;
                        }
                    }
                    else {
                        if (unknown_index != d_children.size()) {
                            multiple_unknown = true;
                            break;
                        }
                        unknown_index = j;
                    }
                }

                if (!has_zero_child) {
                    if (unknown_index == d_children.size()) {
                        if (!multiple_unknown)
                            return Result::CONFLICT;  // all fixed to 1 but output is 0
                    }
                    else if (!multiple_unknown) {
                        result |= d_children.at(unknown_index)->set_fixed(i, false);
                    }
                }
            }
        }
        else {  // DOWN -> UP
            bool bit = true, all_fixed = true;
            for (const auto& child : d_children) {
                if (!child->is_fixed(i)) {
                    all_fixed = false;
                }
                else {
                    bit &= child->get_value(i);
                }
            }

            if (!bit) {
                result |= d_self->set_fixed(i, false);
            }
            else if (all_fixed) {
                result |= d_self->set_fixed(i, true);
            }
        }
    }
    return result;
}

Result
FdpAndOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    // [TODO]
    if (d_self->is_interval_complementary())
        return result;
    for (const auto& child : d_children) {
        if (child->is_interval_complementary())
            continue;
        if (d_self->interval_min().compare(child->interval_max()) > 0)
            return Result::CONFLICT;
        result |= d_self->apply_interval_constraint(
            d_self->interval_min(),
            child->interval_max(),
            false);
    }
    return result;
}

Result FdpAndOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_and);
    ++d_stats.d_num_fdp_and;

    Result result = Result::UNCHANGED;
    result |= fixed_bits_both_way();
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }
    result |= interval_both_way();
    return result;
}

std::vector<uint32_t> FdpAndOperator::implied_by() {
    if (d_self->is_fixed_one(0))
        return {0, 1};
    else if (d_children.at(0)->is_fixed_zero(0))
        return {0};
    else if (d_children.at(1)->is_fixed_zero(0))
        return {1};
    else
        return {};
}
}  // namespace bzla::preprocess::pass::fdp
