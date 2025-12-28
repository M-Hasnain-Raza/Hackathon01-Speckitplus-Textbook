# Phase 1: Citation Strategy — Module 2 Digital Twin

**Date**: 2025-12-28
**Purpose**: Source catalog, APA format examples, citation rules, and audit script specification
**Compliance**: Constitution Principle II (Source-Grounded Facts)

---

## Citation Philosophy

**Core Principle**: Every factual claim in Module 2 documentation MUST be source-grounded with explicit citations.

**What Requires Citation**:
- ✅ Technical facts (e.g., "Gazebo Fortress uses ODE as default physics engine")
- ✅ Version numbers and compatibility (e.g., "ROS 2 Humble requires Python 3.10+")
- ✅ Performance characteristics (e.g., "Typical RTF for TurtleBot3 is 1.5-2.0")
- ✅ API signatures and parameters (e.g., "`gpu_lidar` sensor type accepts `<range>` element")
- ✅ Configuration syntax (e.g., "SDF world files use `<world>` root element")
- ✅ Error messages (e.g., "`Failed to load plugin libgazebo_ros_diff_drive.so`")
- ✅ Academic claims (e.g., "Domain randomization improves sim2real transfer (Tobin et al., 2017)")

**What Does NOT Require Citation**:
- ❌ Conceptual explanations (e.g., "Digital twins enable safe testing before hardware deployment")
- ❌ Learning objectives (e.g., "After this section, you will understand sensor noise models")
- ❌ Analogies and examples (e.g., "Think of Gazebo as a physics sandbox for robots")
- ❌ General robotics principles (e.g., "Sensors provide perception data for robot decision-making")

**Guideline**: If a reader might ask "How do you know this?", it needs a citation. If it's conceptual framing or pedagogical scaffolding, no citation needed.

---

## Source Tier Classification

### Tier 1: Official Documentation (Authoritative, Always Preferred)

**Gazebo**:
- Gazebo Documentation: https://gazebosim.org/docs
  - Fortress: https://gazebosim.org/docs/fortress
  - Garden: https://gazebosim.org/docs/garden
  - Harmonic: https://gazebosim.org/docs/harmonic
  - Tutorials: https://gazebosim.org/docs/fortress/tutorials
  - Sensors: https://gazebosim.org/docs/fortress/sensors
  - Physics: https://gazebosim.org/docs/fortress/physics
- SDF Specification: http://sdformat.org/spec
- Gazebo GitHub (for plugin source code): https://github.com/gazebosim

**ROS 2**:
- ROS 2 Documentation: https://docs.ros.org/en/humble/
  - Tutorials: https://docs.ros.org/en/humble/Tutorials.html
  - Concepts: https://docs.ros.org/en/humble/Concepts.html
  - sensor_msgs Package: https://docs.ros.org/en/humble/p/sensor_msgs/
  - geometry_msgs Package: https://docs.ros.org/en/humble/p/geometry_msgs/
- ros_gz Package: https://github.com/gazebosim/ros_gz
- ros_gz_bridge: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
- ros_gz_sim: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim
- ros_gz_interfaces: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_interfaces

**Unity**:
- Unity Documentation: https://docs.unity3d.com/Manual/
- Unity ROS-TCP-Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector
  - README: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/README.md
- Unity Personal Licensing: https://unity.com/products/unity-personal

### Tier 2: Authoritative Robotics Resources (Community-Maintained, High Quality)

- ROS 2 Design Documentation: https://design.ros2.org/
- Gazebo Community Tutorials: https://community.gazebosim.org/
- TurtleBot3 e-Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- ROS 2 Navigation (Nav2): https://docs.nav2.org/
- MoveIt 2 Documentation: https://moveit.picknik.ai/main/index.html

### Tier 3: Academic Literature (Peer-Reviewed Papers)

**Sim2Real Transfer**:
- **Primary Reference**: Tobin, J., Fong, R., Ray, A., Schneider, J., Zaremba, W., & Abbeel, P. (2017). Domain randomization for transferring deep neural networks from simulation to the real world. *2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 23-30. https://doi.org/10.1109/IROS.2017.8202133
- Additional sim2real papers (to be identified during content writing based on specific claims)

**Robotics Fundamentals**:
- Standard robotics textbooks (Thrun, Burgard, Fox: "Probabilistic Robotics"; Craig: "Introduction to Robotics")

### Sources to AVOID

❌ **Random blog posts** (not authoritative, may have errors)
❌ **Stack Overflow answers** (useful for debugging, not citation-worthy)
❌ **Medium articles** (opinion pieces, not peer-reviewed)
❌ **YouTube tutorials** (helpful for learning, not authoritative sources)
❌ **Reddit discussions** (community knowledge, not verified)

**Exception**: Well-maintained community wikis (e.g., ROS 2 Discourse official posts) are acceptable for Tier 2 when official docs lack detail.

---

## APA Citation Format

Module 2 uses **APA 7th Edition** style for all citations.

### In-Text Citations

**Direct Quote**:
> "Gazebo uses the ODE (Open Dynamics Engine) as its default physics engine for efficient rigid body dynamics" (Gazebo Documentation, 2023, Physics section).

**Paraphrased Fact**:
> The default physics engine in Gazebo Fortress is ODE (Gazebo Documentation, 2023).

**Multiple Sources**:
> Domain randomization has been shown to improve sim2real transfer success rates (Tobin et al., 2017; Peng et al., 2018).

**Web Resources (No Author)**:
> The SDF specification defines the `<world>` element as the root container for simulation environments (SDF Specification, n.d., World section).

### Reference List Examples

**Official Documentation (Web Page)**:
```
Gazebo Documentation. (2023). Physics simulation. Gazebo. https://gazebosim.org/docs/fortress/physics
```

**GitHub Repository (Software)**:
```
Gazebo Team. (2023). ros_gz: Integration of Gazebo and ROS 2 (Version 0.244.10) [Computer software]. GitHub. https://github.com/gazebosim/ros_gz
```

**Journal Article (Academic)**:
```
Tobin, J., Fong, R., Ray, A., Schneider, J., Zaremba, W., & Abbeel, P. (2017). Domain randomization for transferring deep neural networks from simulation to the real world. In 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 23-30). IEEE. https://doi.org/10.1109/IROS.2017.8202133
```

**SDF Specification (Technical Standard)**:
```
Open Source Robotics Foundation. (n.d.). SDF specification 1.9. Retrieved December 28, 2025, from http://sdformat.org/spec?ver=1.9
```

**Unity Documentation**:
```
Unity Technologies. (2023). Unity manual. Unity Documentation. https://docs.unity3d.com/Manual/
```

---

## Citation Workflow for Content Authors

### Step 1: Identify Factual Claims (During Writing)

As you write each section, mark factual claims with `[CITE]` placeholder:

```markdown
Gazebo Fortress uses ODE as its default physics engine [CITE]. The physics step size defaults to 0.001 seconds [CITE], resulting in 1000 physics updates per simulated second [CITE].
```

### Step 2: Add Citations (Immediately After Draft)

Replace `[CITE]` with proper APA in-text citations:

```markdown
Gazebo Fortress uses ODE as its default physics engine (Gazebo Documentation, 2023, Physics section). The physics step size defaults to 0.001 seconds (SDF Specification, n.d., Physics element), resulting in 1000 physics updates per simulated second.
```

### Step 3: Build Reference List (Section Frontmatter)

Add full APA references to frontmatter `sources` field:

```yaml
---
sources:
  - "Gazebo Documentation. (2023). Physics simulation. https://gazebosim.org/docs/fortress/physics"
  - "SDF Specification. (n.d.). Physics element. http://sdformat.org/spec?elem=physics"
---
```

### Step 4: Run Citation Audit Script

```bash
python specs/002-digital-twin-simulation/scripts/citation-audit.sh docs/module-02-digital-twin/
```

Script flags:
- ⚠️ Sentences with technical claims but no citation
- ⚠️ `[CITE]` placeholders not replaced
- ⚠️ Citations in text but missing from frontmatter `sources`

---

## Citation Audit Script Specification

**File**: `specs/002-digital-twin-simulation/scripts/citation-audit.sh`
**Language**: Python (for regex and file parsing)
**Functionality**:

1. **Parse MDX Files**: Extract body content (skip code blocks)
2. **Detect Factual Claims**: Regex patterns for technical assertions:
   - Version numbers: `v\d+\.\d+`, `ROS 2 Humble`, `Gazebo Fortress`
   - Technical parameters: `default`, `typical`, `maximum`, `minimum`
   - Specific values: numbers with units (e.g., "0.001 seconds", "1000 Hz")
   - Error messages: strings in backticks with "error" or "warning"
3. **Check for Citations**: Verify each factual claim has nearby APA citation (within 2 sentences)
4. **Output Report**:
   - File: section name
   - Line number: approximate location of uncited claim
   - Excerpt: 50-character snippet of uncited claim
   - Severity: WARNING (recommendation) or ERROR (must fix)

**Example Output**:
```
Citation Audit Report
====================
File: docs/module-02-digital-twin/02-gazebo-fundamentals.mdx
  [WARNING] Line 45: "Gazebo Fortress uses ODE as default..." - No citation found
  [WARNING] Line 67: "ros_gz_bridge translates Ignition Transport to ROS 2 DDS..." - No citation found

File: docs/module-02-digital-twin/05-lidar-sensors.mdx
  [ERROR] Line 102: "Typical LiDAR range is 10 meters..." - Citation placeholder [CITE] not replaced

Summary:
  Total sections scanned: 14
  Uncited claims (warnings): 8
  Unresolved placeholders (errors): 1
  Citation compliance: 94%
```

**Usage in Workflow**: Run after each section completion (Phase 3-10) and before final polish (Phase 11).

---

## Word Count Validation Script Specification

**File**: `specs/002-digital-twin-simulation/scripts/validate-word-count.sh`
**Language**: Python
**Functionality**:

1. **Parse MDX Files**: Extract body content (exclude frontmatter, code blocks)
2. **Count Words**: Standard word count algorithm (whitespace-separated tokens)
3. **Check Range**: Validate word count is 800-1500 per section (RAG optimization requirement)
4. **Output Report**:
   - Section ID
   - Actual word count
   - Status: ✅ PASS (800-1500), ⚠️ WARN (<800 or >1500), ❌ FAIL (<600 or >2000)

**Example Output**:
```
Word Count Validation Report
=============================
Section 01 (introduction-to-digital-twins.mdx): 1,203 words ✅ PASS
Section 02 (gazebo-fundamentals.mdx): 1,456 words ✅ PASS
Section 03 (physics-simulation.mdx): 753 words ⚠️ WARN (target: 800-1500)
Section 04 (sensor-simulation-overview.mdx): 1,612 words ⚠️ WARN (exceeds 1500)
Section 05 (lidar-sensors.mdx): 1,401 words ✅ PASS

Summary:
  Total sections: 14
  PASS: 10
  WARN: 4
  FAIL: 0
  Average word count: 1,287 words
```

**Usage in Workflow**: Run after each section draft and before final polish (Phase 11).

---

## Frontmatter Source Tracking

**Template for Each Section**:

```yaml
---
id: section-id
title: "Section Title"
sources:
  - "Gazebo Documentation. (2023). Topic. https://gazebosim.org/docs/fortress/topic"
  - "Author, A. (Year). Paper title. Journal, Volume(Issue), pages. https://doi.org/..."
  - "Organization. (n.d.). Resource title. https://example.com"
---
```

**Validation**:
- Every item in `sources` is APA-formatted
- Every in-text citation has corresponding entry in `sources`
- Retrieval dates within 6 months (for "n.d." web resources)

---

## Citation Quality Checklist

Before marking a section complete, verify:

- [ ] All factual claims have APA in-text citations
- [ ] All in-text citations have full references in frontmatter `sources`
- [ ] All sources are Tier 1 (official docs) or Tier 2 (authoritative) or Tier 3 (peer-reviewed)
- [ ] No random blog posts, Stack Overflow, or YouTube links cited
- [ ] Citation audit script passes with 0 errors
- [ ] Conceptual explanations are NOT over-cited (preserve readability)

---

**Status**: ✅ Citation strategy defined
**Next Step**: Implement citation audit and word count validation scripts (Phase 2, T014-T015)
