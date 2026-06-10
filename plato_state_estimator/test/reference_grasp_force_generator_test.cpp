#include <gtest/gtest.h>

#include "plato_state_estimator/reference_grasp_force_generator.hpp"

namespace
{

using plato_state_estimator::ReferenceGraspForceConfig;
using plato_state_estimator::ReferenceGraspForceGenerator;
using plato_state_estimator::SlipState;
using plato_state_estimator::TactileData;

TactileData tactile(
  int contact_state,
  double normal_force_n,
  double shear_x_mm = 0.0,
  double shear_y_mm = 0.0,
  double shear_theta_rad = 0.0)
{
  TactileData data;
  data.contact_state = contact_state;
  data.force_z = normal_force_n;
  data.shear_x = shear_x_mm;
  data.shear_y = shear_y_mm;
  data.shear_theta = shear_theta_rad;
  return data;
}

}  // namespace

TEST(ReferenceGraspForceGeneratorTest, NoContactInvalidatesReference)
{
  ReferenceGraspForceGenerator generator(ReferenceGraspForceConfig{});

  const auto output = generator.update(
    tactile(TactileData::NO_CONTACT, 0.0),
    tactile(TactileData::NO_CONTACT, 0.0),
    0.01);

  EXPECT_EQ(output.slip_state, SlipState::NO_CONTACT);
  EXPECT_FALSE(output.reference_valid);
  EXPECT_DOUBLE_EQ(output.target_normal_force_n, 0.0);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_min_n, 0.0);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_avg_n, 0.0);
}

TEST(ReferenceGraspForceGeneratorTest, OneContactIsPartialAndInvalid)
{
  ReferenceGraspForceGenerator generator(ReferenceGraspForceConfig{});

  const auto output = generator.update(
    tactile(TactileData::ENOUGH_CONTACTS, 2.0),
    tactile(TactileData::NO_CONTACT, 0.0),
    0.01);

  EXPECT_EQ(output.slip_state, SlipState::PARTIAL_CONTACT);
  EXPECT_FALSE(output.reference_valid);
  EXPECT_DOUBLE_EQ(output.target_normal_force_n, 0.0);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_min_n, 0.0);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_avg_n, 1.0);
}

TEST(ReferenceGraspForceGeneratorTest, WeakTwoSidedContactIsPartialAndInvalid)
{
  ReferenceGraspForceGenerator generator(ReferenceGraspForceConfig{});

  const auto output = generator.update(
    tactile(TactileData::ENOUGH_CONTACTS, 0.25),
    tactile(TactileData::ENOUGH_CONTACTS, 2.0),
    0.01);

  EXPECT_EQ(output.slip_state, SlipState::PARTIAL_CONTACT);
  EXPECT_FALSE(output.reference_valid);
  EXPECT_DOUBLE_EQ(output.target_normal_force_n, 0.0);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_min_n, 0.25);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_avg_n, 1.125);
}

TEST(ReferenceGraspForceGeneratorTest, BothContactsSmallShearUsesBaseForce)
{
  ReferenceGraspForceGenerator generator(ReferenceGraspForceConfig{});

  const auto output = generator.update(
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 0.05, 0.0, 0.01),
    tactile(TactileData::ENOUGH_CONTACTS, 3.0, 0.05, 0.0, 0.01),
    0.01);

  EXPECT_EQ(output.slip_state, SlipState::STABLE_GRASP);
  EXPECT_TRUE(output.reference_valid);
  EXPECT_DOUBLE_EQ(output.target_normal_force_n, 1.0);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_min_n, 2.0);
  EXPECT_DOUBLE_EQ(output.measured_normal_force_avg_n, 2.5);
}

TEST(ReferenceGraspForceGeneratorTest, TranslationalShearIncreasesTargetForce)
{
  ReferenceGraspForceGenerator generator(ReferenceGraspForceConfig{});

  const auto output = generator.update(
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 1.0, 0.0, 0.0),
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 1.0, 0.0, 0.0),
    0.01);

  EXPECT_EQ(output.slip_state, SlipState::TRANSLATIONAL_SLIP);
  EXPECT_TRUE(output.reference_valid);
  EXPECT_GT(output.target_normal_force_n, 1.0);
  EXPECT_DOUBLE_EQ(output.shear_translation_mm, 1.0);
}

TEST(ReferenceGraspForceGeneratorTest, RotationalShearIncreasesTargetForce)
{
  ReferenceGraspForceGenerator generator(ReferenceGraspForceConfig{});

  const auto output = generator.update(
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 0.0, 0.0, 0.10),
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 0.0, 0.0, 0.10),
    0.01);

  EXPECT_EQ(output.slip_state, SlipState::ROTATIONAL_SLIP);
  EXPECT_TRUE(output.reference_valid);
  EXPECT_GT(output.target_normal_force_n, 1.0);
  EXPECT_DOUBLE_EQ(output.shear_rotation_rad, 0.10);
}

TEST(ReferenceGraspForceGeneratorTest, TargetForceIsClampedToSafetyLimit)
{
  ReferenceGraspForceConfig config;
  config.max_force_limit_n = 1.25;
  ReferenceGraspForceGenerator generator(config);

  const auto output = generator.update(
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 10.0, 0.0, 1.0),
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 10.0, 0.0, 1.0),
    0.01);

  EXPECT_TRUE(output.reference_valid);
  EXPECT_DOUBLE_EQ(output.target_normal_force_n, 1.25);
}

TEST(ReferenceGraspForceGeneratorTest, SignConfigAffectsAveragedShear)
{
  ReferenceGraspForceConfig config;
  config.tactile1_shear_x_sign = -1.0;
  ReferenceGraspForceGenerator generator(config);

  const auto output = generator.update(
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 1.0, 0.0, 0.0),
    tactile(TactileData::ENOUGH_CONTACTS, 2.0, 1.0, 0.0, 0.0),
    0.01);

  EXPECT_EQ(output.slip_state, SlipState::STABLE_GRASP);
  EXPECT_TRUE(output.reference_valid);
  EXPECT_DOUBLE_EQ(output.shear_translation_mm, 0.0);
  EXPECT_DOUBLE_EQ(output.target_normal_force_n, 1.0);
}
