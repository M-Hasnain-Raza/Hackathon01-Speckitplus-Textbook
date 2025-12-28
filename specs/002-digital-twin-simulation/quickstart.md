# Phase 1: Quickstart Guide — Module 2 Implementation

**Date**: 2025-12-28
**Audience**: Content authors implementing Module 2 documentation
**Purpose**: Step-by-step guide for creating MDX sections following Module 2 architecture

---

## Prerequisites

Before starting content creation, ensure you have:

- ✅ Completed Phase 0 (research.md): All 6 research tasks finished
- ✅ Reviewed Phase 1 artifacts:
  - `section-structure.md`: Section outlines and learning objectives
  - `architecture-diagrams.md`: 4 Mermaid diagrams ready for embedding
  - `citation-strategy.md`: Citation rules and source catalog
  - `quickstart.md`: This guide
- ✅ Development environment:
  - Node.js 18+ installed
  - Docusaurus 3.1.0 dependencies installed (`npm install`)
  - ROS 2 Humble + Gazebo Fortress environment for code testing (optional but recommended)
- ✅ Git branch: `002-digital-twin-simulation` checked out

---

## Content Creation Workflow (Per Section)

### Step 1: Create MDX File

**Location**: `docs/module-02-digital-twin/`
**Naming**: `##-section-id.mdx` (e.g., `01-introduction-to-digital-twins.mdx`)

**Template**:

```mdx
---
id: section-id
title: "Section Title"
sidebar_label: "##: Short Label"
sidebar_position: ##
description: "1-2 sentence section summary for search and previews"
keywords: [keyword1, keyword2, keyword3, ...]
sources:
  - "Citation 1 in APA format"
  - "Citation 2 in APA format"
learning_objectives:
  - "Objective 1"
  - "Objective 2"
prerequisites: ["prerequisite-section-id", "module-01"]
estimated_time: "XX minutes"
---

# {frontmatter.title}

## Introduction

[Motivate the topic, explain why it matters, preview what will be covered]

## [Subsection 1]

[Content with code examples, diagrams, explanations]

[More subsections...]

## Summary

[Key takeaways, what reader should now be able to do]

## Next Steps

[Link to next section or suggest practice exercises]
```

### Step 2: Write Content (800-1500 words)

**Structure** (Consistent Template):
1. **Introduction** (100-150 words): Motivate topic, connect to prior knowledge
2. **Concept Explanation** (300-500 words): Core idea with clear definitions
3. **Code Example** (200-300 words): Practical demonstration with annotated code
4. **Practice Exercise** (100-200 words): Hands-on task for reader
5. **Summary** (100-150 words): Key takeaways, reinforce learning objectives

**Writing Guidelines**:
- **Tone**: Instructional but conversational (second person "you")
- **Complexity**: Advanced undergraduate/graduate level
- **Code Examples**: Python (rclpy) for ROS 2, XML for SDF/URDF, Bash for commands
- **Visuals**: Embed Mermaid diagrams, reference screenshots (to be captured in Phase 5)
- **Cross-References**: Link to Module 1 concepts explicitly (e.g., "As you learned in [Module 1: ROS 2 Topics](/module-01-ros2/topics)...")

**Avoid**:
- ❌ Out-of-scope content (C++ plugins, Unity shaders, HIL, multi-robot, RTOS, commercial platforms, RL theory)
- ❌ Passive voice ("The sensor is configured by the user" → "You configure the sensor")
- ❌ Unexplained acronyms (always define on first use)

### Step 3: Add Citations (APA Format)

**As You Write**:
1. Identify factual claims (technical details, version numbers, API signatures, performance data)
2. Add in-text citation: `(Source, Year, Section)`
3. Add full reference to frontmatter `sources` list

**Example**:

```mdx
Gazebo Fortress uses ODE (Open Dynamics Engine) as its default physics engine (Gazebo Documentation, 2023, Physics section). The physics step size defaults to 0.001 seconds (SDF Specification, n.d., Physics element), enabling 1000 physics updates per simulated second.

---
sources:
  - "Gazebo Documentation. (2023). Physics simulation. https://gazebosim.org/docs/fortress/physics"
  - "SDF Specification. (n.d.). Physics element. http://sdformat.org/spec?elem=physics"
---
```

**Citation Rules**:
- Every factual claim = citation
- Conceptual explanations = no citation
- Use Tier 1 sources (official docs) when possible
- Avoid blog posts, Stack Overflow, YouTube

### Step 4: Test Code Examples

**For Sections with Code** (Sections 02-03, 05-07, 08, 10, 12):

1. **Copy code from MDX** to test environment
2. **Run in ROS 2 Humble + Gazebo Fortress**:
   ```bash
   # Example: Test LiDAR sensor URDF
   gz sim -s sensor_test.sdf
   ros2 topic echo /scan
   ```
3. **Verify output matches expectations** (topics publish, no errors)
4. **Add version comment to code**:
   ```python
   # Tested with ROS 2 Humble + Gazebo Fortress
   # Date: 2025-12-28
   ```
5. **Capture screenshots** (for Phase 5):
   - Save to `static/img/module-02/section-##-description.png`
   - Add descriptive alt text in MDX

### Step 5: Embed Diagrams (If Applicable)

**For Sections with Mermaid Diagrams** (Sections 01, 02, 10, 11):

1. **Copy Mermaid code** from `architecture-diagrams.md`
2. **Embed in MDX**:
   ````mdx
   ```mermaid
   graph TD
       A[Start] --> B[Process]
       B --> C[End]
   ```
   ````
3. **Add explanation** after diagram (what does it show, why does it matter)

### Step 6: Validate Section

**Automated Checks**:

```bash
# Word count (target: 800-1500)
python specs/002-digital-twin-simulation/scripts/validate-word-count.sh docs/module-02-digital-twin/01-introduction-to-digital-twins.mdx

# Citation audit (0 errors)
python specs/002-digital-twin-simulation/scripts/citation-audit.sh docs/module-02-digital-twin/01-introduction-to-digital-twins.mdx

# Docusaurus build
npm run build
```

**Manual Checks**:
- [ ] Frontmatter complete (all required fields)
- [ ] Learning objectives met (section content addresses each objective)
- [ ] Word count 800-1500
- [ ] Code examples tested (if any)
- [ ] Citations for all factual claims
- [ ] No out-of-scope content
- [ ] Cross-references to Module 1 correct
- [ ] Mermaid diagrams render (if any)

### Step 7: Preview in Dev Server

```bash
npm start
```

**Visual Review**:
- [ ] Navigation works (sidebar link active, next/previous buttons)
- [ ] Code syntax highlighting correct (python, bash, xml, yaml)
- [ ] Mermaid diagrams render without errors
- [ ] Screenshots display (alt text present)
- [ ] Cross-references link correctly

### Step 8: Mark Task Complete

Update `specs/002-digital-twin-simulation/tasks.md`:

```markdown
- [x] T016 [P] [US1] Write Section 01: Introduction to Digital Twins in `docs/module-02-digital-twin/01-introduction-to-digital-twins.mdx`
```

---

## File Structure Reference

```
hackathon01-Textbook/
├── docs/
│   └── module-02-digital-twin/
│       ├── 01-introduction-to-digital-twins.mdx
│       ├── 02-gazebo-fundamentals.mdx
│       ├── 03-physics-simulation.mdx
│       ├── 04-sensor-simulation.mdx
│       ├── 05-lidar-sensors.mdx
│       ├── 06-depth-cameras.mdx
│       ├── 07-imu-sensors.mdx
│       ├── 08-custom-worlds.mdx
│       ├── 09-unity-visualization.mdx
│       ├── 10-gazebo-unity-integration.mdx
│       ├── 11-sim2real-transfer.mdx
│       ├── 12-programmatic-control.mdx
│       ├── 13-performance-troubleshooting.mdx
│       └── 14-conclusion.mdx
├── static/
│   └── img/
│       └── module-02/
│           ├── section-01-workflow-diagram.png
│           ├── section-05-lidar-rviz.png
│           ├── section-06-camera-pointcloud.png
│           ├── section-07-imu-orientation.png
│           ├── section-08-custom-world.png
│           ├── section-09-unity-scene.png
│           └── section-10-gazebo-unity-sync.png
├── sidebars.js  # Updated with Module 2 navigation
├── specs/
│   └── 002-digital-twin-simulation/
│       ├── spec.md
│       ├── plan.md
│       ├── tasks.md
│       ├── research.md
│       ├── section-structure.md
│       ├── architecture-diagrams.md
│       ├── citation-strategy.md
│       ├── quickstart.md
│       ├── checklists/
│       │   └── requirements.md
│       └── scripts/
│           ├── citation-audit.sh
│           └── validate-word-count.sh
└── package.json
```

---

## Code Example Template

**Python (rclpy) Example**:

```python
# Tested with ROS 2 Humble
# Date: 2025-12-28

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanProcessor(Node):
    """Simple node to process LiDAR scans."""

    def __init__(self):
        super().__init__('scan_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        """Process incoming LaserScan messages."""
        min_range = min(msg.ranges)
        self.get_logger().info(f'Minimum range: {min_range:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**URDF/SDF Example**:

```xml
<!-- Tested in Gazebo Fortress -->
<!-- Date: 2025-12-28 -->

<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.1 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
  <topic>scan</topic>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
</sensor>
```

**Bash Command Example**:

```bash
# Launch Gazebo with TurtleBot3 world (ROS 2 Humble)
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Verify sensor topics are publishing
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

---

## Common Pitfalls to Avoid

1. **Over-Citation**: Don't cite every sentence. Conceptual explanations don't need citations.
2. **Under-Citation**: Every technical fact needs a source. When in doubt, cite.
3. **Out-of-Scope Creep**: Stick to approved scope (no C++ plugins, Unity shaders, etc.)
4. **Copy-Paste from Docs**: Paraphrase official documentation, don't copy verbatim (copyright)
5. **Broken Cross-References**: Verify Module 1 section IDs before linking
6. **Untested Code**: Always run code examples in target environment before publishing
7. **Missing Alt Text**: Every image and diagram needs descriptive alt text (accessibility)
8. **Inconsistent Terminology**: Use "Gazebo Fortress" (not "Ignition Fortress" or "New Gazebo")

---

## MVP Implementation Strategy

**MVP Scope** (User Stories 1,2,3,6 - Priority P1):
- Section 01: Introduction to Digital Twins
- Sections 02-03: Gazebo Setup
- Sections 04-07: Sensors (LiDAR, Camera, IMU)
- Section 11: Sim2Real Transfer

**Total**: 8 sections, ~10,750 words

**Implementation Order**:
1. **Phase 1-2** (Setup + Foundational): Complete all artifacts and diagrams FIRST
2. **Phase 3**: Section 01 (conceptual foundation)
3. **Phase 4**: Sections 02-03 (Gazebo fundamentals - prerequisite for sensors)
4. **Phase 5**: Sections 04-07 (sensors in parallel after Section 04 overview)
5. **Phase 8**: Section 11 (sim2real - completes MVP)
6. **STOP and VALIDATE**: Test MVP with beta readers
7. **Deploy/Demo**: Publish MVP to staging environment
8. **Phases 6-7, 9-10**: P2 content (custom worlds, Unity, AI, performance, conclusion)
9. **Phase 11**: Polish and final QA

---

## Quality Assurance Checklist (Before Section Merge)

- [ ] Frontmatter complete and valid
- [ ] Learning objectives addressed in content
- [ ] Word count 800-1500 (RAG-optimized)
- [ ] All factual claims cited (APA format)
- [ ] All citations in frontmatter `sources`
- [ ] No Tier 3 sources without Tier 1/2 alternatives
- [ ] Code examples tested in target environment
- [ ] Screenshots captured and alt text added
- [ ] Mermaid diagrams render without errors
- [ ] Cross-references to Module 1 verified
- [ ] No out-of-scope content (7 exclusions checked)
- [ ] Docusaurus build succeeds (`npm run build`)
- [ ] Preview looks correct in dev server (`npm start`)
- [ ] Task marked complete in tasks.md

---

## Getting Help

**Technical Questions**:
- Gazebo: https://community.gazebosim.org/
- ROS 2: https://discourse.ros.org/
- Unity ROS: https://github.com/Unity-Technologies/ROS-TCP-Connector/issues

**Documentation Style**:
- Docusaurus: https://docusaurus.io/docs
- Markdown/MDX: https://mdxjs.com/docs/
- Mermaid: https://mermaid.js.org/intro/

**Project Specific**:
- Constitution: `.specify/memory/constitution.md`
- Spec: `specs/002-digital-twin-simulation/spec.md`
- Plan: `specs/002-digital-twin-simulation/plan.md`
- Tasks: `specs/002-digital-twin-simulation/tasks.md`

---

**Status**: ✅ Quickstart guide complete
**Ready for**: Phase 3 content creation (Section 01: Introduction to Digital Twins)
