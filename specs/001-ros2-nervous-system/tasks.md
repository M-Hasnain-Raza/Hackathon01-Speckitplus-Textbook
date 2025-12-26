# Tasks: ROS 2 — The Robotic Nervous System Documentation Module

**Input**: Design documents from `/specs/001-ros2-nervous-system/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: No test tasks included - specification explicitly states conceptual questions only, no automated testing required.

**Organization**: Tasks are grouped by user story to enable independent implementation and validation of each story's learning outcomes.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation content**: `docs/module-01-ros2-nervous-system/`
- **Planning artifacts**: `specs/001-ros2-nervous-system/`
- **Diagram sources**: `specs/001-ros2-nervous-system/diagrams/`
- **Code examples**: `specs/001-ros2-nervous-system/code-examples/`

---

## Phase 1: Setup (Documentation Infrastructure)

**Purpose**: Project initialization and documentation structure setup

- [X] T001 Create documentation directory structure at docs/module-01-ros2-nervous-system/
- [X] T002 Create Docusaurus category configuration file docs/module-01-ros2-nervous-system/_category_.json
- [X] T003 [P] Create diagram source directory specs/001-ros2-nervous-system/diagrams/
- [X] T004 [P] Create code examples directory specs/001-ros2-nervous-system/code-examples/
- [X] T005 [P] Create validation directory specs/001-ros2-nervous-system/validation/

---

## Phase 2: Research & Design (Foundational Prerequisites)

**Purpose**: Research ROS 2 documentation sources and create reusable templates/examples

**⚠️ CRITICAL**: No content writing can begin until research is complete

- [X] T006.1 Research ROS 2 core concepts (graph, nodes, topics, services, actions) from official docs - 2h est.
- [X] T006.2 Research rclpy API documentation (Node, Publisher, Subscriber, Service, Action APIs) - 2h est.
- [X] T006.3 Research ROS 2 message types (std_msgs, geometry_msgs, sensor_msgs) - 1.5h est.
- [X] T006.4 Research URDF format for robot description (Section 10) - 1.5h est.
- [X] T006.5 Research Docusaurus features (MDX, tabs, admonitions, code blocks) - 1h est.
- [X] T006.6 Research AI integration patterns and RAG-friendly documentation structure - 1.5h est.
- [X] T006.7 Research Mermaid diagram syntax (sequence, flow, architecture diagrams) - 1h est.
- [X] T006.8 Consolidate all research findings into specs/001-ros2-nervous-system/research.md
- [ ] T007 Create detailed section structure outline in specs/001-ros2-nervous-system/section-structure.md
- [ ] T008 [P] Create Mermaid diagram for nervous system analogy in specs/001-ros2-nervous-system/diagrams/nervous-system-analogy.mmd
- [ ] T009 [P] Create Mermaid diagram for node architecture in specs/001-ros2-nervous-system/diagrams/node-architecture.mmd
- [ ] T010 [P] Create Mermaid diagram for topic flow in specs/001-ros2-nervous-system/diagrams/topic-flow.mmd - will be paired with 2 code examples (18-20 lines each)
- [ ] T011 [P] Create Mermaid diagram for service flow in specs/001-ros2-nervous-system/diagrams/service-flow.mmd - will be paired with 2 code examples (18-20 lines each)
- [ ] T012 [P] Create Mermaid diagram for message structure in specs/001-ros2-nervous-system/diagrams/message-structure.mmd - will be paired with 2 code examples (18-20 lines each)
- [ ] T013 [P] Create Mermaid diagram for AI agent integration in specs/001-ros2-nervous-system/diagrams/ai-agent-integration.mmd - will be paired with 2 code examples (18-20 lines each) - addresses EC-004
- [ ] T014 [P] Create Mermaid diagram for rclpy lifecycle in specs/001-ros2-nervous-system/diagrams/rclpy-lifecycle.mmd - will be paired with 3 code examples (18-20 lines each) - addresses EC-005
- [ ] T015 [P] Create Mermaid diagram for end-to-end message flow in specs/001-ros2-nervous-system/diagrams/end-to-end-flow.mmd
- [ ] T016 [P] Create Mermaid diagram for URDF arm example in specs/001-ros2-nervous-system/diagrams/urdf-arm-example.mmd - addresses EC-002 and EC-003
- [ ] T017 [P] Create basic node initialization code example in specs/001-ros2-nervous-system/code-examples/node-basic.py
- [ ] T018 [P] Create publisher code example in specs/001-ros2-nervous-system/code-examples/publisher-basic.py
- [ ] T019 [P] Create subscriber code example in specs/001-ros2-nervous-system/code-examples/subscriber-complete.py
- [ ] T020 [P] Create service call code example in specs/001-ros2-nervous-system/code-examples/service-call.py
- [ ] T021 [P] Create Twist message code example in specs/001-ros2-nervous-system/code-examples/message-twist.py
- [ ] T022 [P] Create AI agent skeleton code example in specs/001-ros2-nervous-system/code-examples/ai-agent-skeleton.py
- [ ] T023 [P] Create complete publisher code example in specs/001-ros2-nervous-system/code-examples/publisher-complete.py
- [ ] T024 [P] Create AI arm command code example in specs/001-ros2-nervous-system/code-examples/ai-arm-command.py
- [ ] T025 [P] Create URDF arm snippet in specs/001-ros2-nervous-system/code-examples/urdf-arm-snippet.xml

**Checkpoint**: Research complete, diagrams and code examples ready - content writing can now begin in parallel

---

## Phase 3: User Story 1 - Understanding ROS 2's Role in Physical AI (Priority: P1) 🎯 MVP

**Goal**: Deliver introductory sections explaining ROS 2's role using nervous system analogy

**Independent Test**: Reader can answer "What role does ROS 2 play in a physical AI system?" and "What problem does it solve?" after reading Sections 01-02

### Implementation for User Story 1

- [ ] T026 [P] [US1] Write Section 01: Introduction in docs/module-01-ros2-nervous-system/01-introduction.md - Validation: verify terminology standards compliance (ROS 2 with space, full Publisher/Subscriber terms), word count ≤600, readability grade 10-12, all ROS 2 factual claims cited
- [ ] T027 [P] [US1] Write Section 02: ROS 2 Architecture Overview in docs/module-01-ros2-nervous-system/02-ros2-architecture.md - Content review: includes distributed vs. monolithic comparison, explains node communication, targets 600-800 words, maintains consistent terminology, all technical claims source-grounded
- [ ] T028 [US1] Embed nervous system analogy diagram (from T008) in Section 01 - Diagram validation: renders in Docusaurus without errors, includes alt text, uses correct Mermaid syntax, labels are clear and support "robotic nervous system" analogy, flow is logical (brain→nerves→muscles mapping)
- [ ] T029 [US1] Embed node architecture diagram (from T009) in Section 02
- [ ] T030 [US1] Add conceptual questions to Section 01
- [ ] T031 [US1] Add conceptual questions to Section 02
- [ ] T032 [US1] Validate Section 01 meets 400-600 word target and advanced undergraduate reading level
- [ ] T033 [US1] Validate Section 02 meets 600-800 word target and includes distributed vs. monolithic comparison
- [ ] T034 [US1] Verify frontmatter metadata (title, description, keywords, learning_outcome) in both sections
- [ ] T035 [US1] Verify all factual claims in Sections 01-02 have citations to official ROS 2 docs

**Checkpoint**: User Story 1 complete - Sections 01-02 independently readable and answer foundational "why ROS 2?" questions

---

## Phase 4: User Story 3 - Learning ROS 2 Core Concepts (Priority: P2)

**Goal**: Deliver deep-dive sections on ROS 2 primitives (nodes, topics, services, messages)

**Independent Test**: Reader can define nodes, topics, services, messages and provide humanoid robotics examples for each

### Implementation for User Story 3

- [ ] T036 [P] [US3] Write Section 03: ROS 2 Nodes in docs/module-01-ros2-nervous-system/03-nodes.md
- [ ] T037 [P] [US3] Write Section 04: ROS 2 Topics in docs/module-01-ros2-nervous-system/04-topics.md
- [ ] T038 [P] [US3] Write Section 05: ROS 2 Services in docs/module-01-ros2-nervous-system/05-services.md
- [ ] T039 [P] [US3] Write Section 06: ROS 2 Messages in docs/module-01-ros2-nervous-system/06-messages.md
- [ ] T040 [US3] Embed node initialization code example (from T017) in Section 03
- [ ] T041 [US3] Embed publisher code example (from T018) in Section 04
- [ ] T042 [US3] Embed service call code example (from T020) in Section 05
- [ ] T043 [US3] Embed Twist message code example (from T021) in Section 06
- [ ] T044 [US3] Embed topic flow diagram (from T010) in Section 04
- [ ] T045 [US3] Embed service flow diagram (from T011) in Section 05
- [ ] T046 [US3] Embed message structure diagram (from T012) in Section 06
- [ ] T047 [US3] Add conceptual questions to Section 03 (e.g., "Give three examples of nodes in humanoid robot")
- [ ] T048 [US3] Add conceptual questions to Section 04 (e.g., "When should you use topics vs. services?")
- [ ] T049 [US3] Add conceptual questions to Section 05 (e.g., "Provide two scenarios where services are more appropriate")
- [ ] T050 [US3] Add conceptual questions to Section 06 (e.g., "Why use typed messages vs. generic data?")
- [ ] T051 [US3] Create comparison table for topics vs. services in Section 04 or 05
- [ ] T052 [US3] Validate Section 03 meets 600-800 word target with humanoid robotics examples
- [ ] T053 [US3] Validate Section 04 meets 700-900 word target with pub/sub pattern explanation
- [ ] T054 [US3] Validate Section 05 meets 600-800 word target with synchronous pattern explanation
- [ ] T055 [US3] Validate Section 06 meets 600-800 word target with standard message types listed
- [ ] T056 [US3] Verify all code examples are 10-20 lines with inline comments
- [ ] T057 [US3] Verify frontmatter metadata in Sections 03-06
- [ ] T058 [US3] Verify all factual claims about ROS 2 APIs have citations

**Checkpoint**: User Story 3 complete - Sections 03-06 provide comprehensive ROS 2 primitives knowledge independently

---

## Phase 5: User Story 4 - Understanding Python/rclpy Integration (Priority: P2)

**Goal**: Deliver rclpy integration sections showing how Python AI agents connect to ROS 2

**Independent Test**: Reader can write basic rclpy publisher/subscriber code within 15 minutes

### Implementation for User Story 4

- [ ] T059 [P] [US4] Write Section 07: Connecting AI Agents to ROS 2 in docs/module-01-ros2-nervous-system/07-ai-agent-integration.md
- [ ] T060 [P] [US4] Write Section 08: rclpy Basics in docs/module-01-ros2-nervous-system/08-rclpy-basics.md
- [ ] T061 [US4] Embed AI agent skeleton code example (from T022) in Section 07
- [ ] T062 [US4] Embed complete publisher code example (from T023) in Section 08
- [ ] T063 [US4] Embed complete subscriber code example (from T019) in Section 08
- [ ] T064 [US4] Embed AI agent integration diagram (from T013) in Section 07
- [ ] T065 [US4] Embed rclpy lifecycle diagram (from T014) in Section 08
- [ ] T066 [US4] Add conceptual questions to Section 07 (e.g., "How does an AI agent become part of ROS 2 system?")
- [ ] T067 [US4] Add conceptual questions to Section 08 (e.g., "What does rclpy.spin() do and why necessary?")
- [ ] T068 [US4] Validate Section 07 meets 600-800 word target with high-level integration flow
- [ ] T069 [US4] Validate Section 08 meets 800-1000 word target with rclpy.init(), create_node(), spin(), shutdown() explained
- [ ] T070 [US4] Verify code examples in Section 08 are complete (18-20 lines) with initialization and cleanup
- [ ] T071 [US4] Verify frontmatter metadata in Sections 07-08
- [ ] T072 [US4] Verify rclpy API calls have citations to official rclpy documentation

**Checkpoint**: User Story 4 complete - Sections 07-08 enable readers to write basic rclpy code independently

---

## Phase 6: User Story 2 - Tracing Message Flow from AI to Actuation (Priority: P1)

**Goal**: Deliver end-to-end message flow tracing section connecting all concepts

**Independent Test**: Reader can trace "AI decides to raise left arm" through nodes, topics, messages to actuator

**Note**: This is P1 but sequenced after US3 and US4 because it synthesizes their concepts

### Implementation for User Story 2

- [ ] T073 [US2] Write Section 09: Tracing Message Flow in docs/module-01-ros2-nervous-system/09-message-flow.md
- [ ] T074 [US2] Embed AI arm command code example (from T024) in Section 09
- [ ] T075 [US2] Embed end-to-end message flow diagram (from T015) in Section 09
- [ ] T076 [US2] Add step-by-step trace example for "raise left arm" scenario in Section 09
- [ ] T077 [US2] Add debugging commands (ros2 topic echo, ros2 node list) explanation in Section 09
- [ ] T078 [US2] Add conceptual questions to Section 09 (e.g., "Trace message flow for 'walk forward'")
- [ ] T079 [US2] Validate Section 09 meets 800-1000 word target with concrete example tracing
- [ ] T080 [US2] Verify Section 09 references but doesn't duplicate content from Sections 03-08
- [ ] T081 [US2] Verify frontmatter metadata in Section 09
- [ ] T082 [US2] Verify all factual claims have citations

**Checkpoint**: User Story 2 complete - Section 09 synthesizes all ROS 2 concepts into end-to-end flow tracing

---

## Phase 7: User Story 5 - Grasping URDF Concepts for Humanoid Robots (Priority: P3)

**Goal**: Deliver URDF introduction section for robot structure description

**Independent Test**: Reader can identify links and joints in URDF snippet with 100% accuracy

### Implementation for User Story 5

- [ ] T083 [US5] Write Section 10: URDF Intro in docs/module-01-ros2-nervous-system/10-urdf-intro.md
- [ ] T084 [US5] Embed URDF arm snippet XML example (from T025) in Section 10
- [ ] T085 [US5] Embed URDF arm example diagram (from T016) in Section 10
- [ ] T086 [US5] Add explanation of links (rigid bodies) and joints (connections) in Section 10
- [ ] T087 [US5] Add joint types (revolute, prismatic, fixed) explanation in Section 10
- [ ] T088 [US5] Add joint limits (position, velocity, effort) explanation in Section 10
- [ ] T089 [US5] Add conceptual questions to Section 10 (e.g., "What are two main elements in URDF file?")
- [ ] T090 [US5] Validate Section 10 meets 600-800 word target and stays conceptual (no full URDF tutorial)
- [ ] T091 [US5] Verify frontmatter metadata in Section 10
- [ ] T092 [US5] Verify URDF specification citations

**Checkpoint**: User Story 5 complete - Section 10 provides URDF basics independently

---

## Phase 8: Module Summary and Integration

**Purpose**: Create module landing page, summary section, and validate integration

- [ ] T093 Write module landing page docs/module-01-ros2-nervous-system/index.md
- [ ] T094 Write Section 11: Module Summary in docs/module-01-ros2-nervous-system/11-summary.md
- [ ] T095 Add recap of key concepts (3-5 bullet points) in Section 11
- [ ] T096 Add "What's NOT in Module 1" list in Section 11 (installation, hardware, simulation, advanced topics)
- [ ] T097 Add preview of future modules in Section 11
- [ ] T098 Add 5-7 comprehensive review questions covering all sections in Section 11
- [ ] T099 Validate Section 11 has no code examples or diagrams (summary only)
- [ ] T100 Verify module landing page includes module overview and section navigation
- [ ] T100A Validate diagram quality against FR-007 requirements - verify all diagrams (T008-T016) render correctly in Docusaurus, include descriptive alt text, use correct Mermaid syntax, and present clear visual flow with readable labels (1h est.)

---

## Phase 9: Polish & Quality Validation

**Purpose**: Final validation across all sections and cross-cutting quality checks

- [ ] T101 [P] Run citation completeness check - create specs/001-ros2-nervous-system/validation/citation-check.md
- [ ] T102 [P] Run scope compliance check - create specs/001-ros2-nervous-system/validation/scope-compliance.md
- [ ] T103 [P] Run accessibility check - create specs/001-ros2-nervous-system/validation/accessibility-check.md
- [ ] T104 Validate all sections (01-11) follow heading hierarchy (H1 → H2 → H3)
- [ ] T105 Validate all internal links between sections work correctly
- [ ] T106 Validate all code examples have language tags for syntax highlighting
- [ ] T107 Validate all diagrams have alt text for accessibility
- [ ] T108 Run Docusaurus build verification (no errors or warnings)
- [ ] T109 Verify module reading time is 45-60 minutes (total word count ~5000-7000 words)
- [ ] T110 Verify each section is independently retrievable with frontmatter metadata
- [ ] T111 [P] Run automated scope violation check (grep for "installation", "Gazebo", "Isaac Sim", etc.)
- [ ] T112 [P] Verify all Python code examples follow PEP 8 style guidelines
- [ ] T113 Final review against all FR-001 through FR-013 functional requirements
- [ ] T114 Final review against all SC-001 through SC-009 success criteria

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Research & Design (Phase 2)**: Depends on Setup completion - BLOCKS all content writing
- **User Stories (Phases 3-7)**: All depend on Phase 2 completion
  - Phase 3 (US1): Can start after Phase 2 - No dependencies on other stories
  - Phase 4 (US3): Can start after Phase 2 - Independent of other stories
  - Phase 5 (US4): Can start after Phase 2 - Independent of other stories
  - Phase 6 (US2): Recommended after Phases 3, 4, 5 for best synthesis (but technically independent)
  - Phase 7 (US5): Can start after Phase 2 - Independent of other stories
- **Module Summary (Phase 8)**: Depends on all user story phases (3-7) being complete
- **Polish (Phase 9)**: Depends on all content (Phases 3-8) being complete

### User Story Dependencies

- **User Story 1 (P1 - Sections 01-02)**: Foundation layer - explains "why ROS 2"
- **User Story 3 (P2 - Sections 03-06)**: Core concepts layer - explains "what ROS 2 is"
- **User Story 4 (P2 - Sections 07-08)**: Integration layer - explains "how to use rclpy"
- **User Story 2 (P1 - Section 09)**: Synthesis layer - combines concepts from US1, US3, US4
- **User Story 5 (P3 - Section 10)**: Supporting concept - independent of flow tracing

**Recommended Order** (for maximum coherence):
1. Phase 2 (Research) - MUST be complete first
2. Phase 3 (US1) - Foundation
3. Phases 4 and 5 in parallel (US3 and US4) - Core concepts and integration
4. Phase 6 (US2) - Synthesis (after US1, US3, US4 provide context)
5. Phase 7 (US5) - Can be done anytime after Phase 2
6. Phase 8 - Summary
7. Phase 9 - Validation

### Parallel Opportunities

- **Phase 1 Setup**: All tasks (T001-T005) can run in parallel
- **Phase 2 Research**:
  - Diagrams (T008-T016) can all run in parallel after research (T006)
  - Code examples (T017-T025) can all run in parallel after research (T006)
- **Phase 3 (US1)**: T026 and T027 can run in parallel (different sections)
- **Phase 4 (US3)**: T036-T039 can run in parallel (different sections)
- **Phase 5 (US4)**: T059 and T060 can run in parallel (different sections)
- **Phase 9 Polish**: T101-T103 (validation checks) and T111-T112 (automated checks) can run in parallel

---

## Parallel Example: Phase 4 (User Story 3)

```bash
# Launch all section writing for US3 together:
Task: "Write Section 03: ROS 2 Nodes in docs/module-01-ros2-nervous-system/03-nodes.md"
Task: "Write Section 04: ROS 2 Topics in docs/module-01-ros2-nervous-system/04-topics.md"
Task: "Write Section 05: ROS 2 Services in docs/module-01-ros2-nervous-system/05-services.md"
Task: "Write Section 06: ROS 2 Messages in docs/module-01-ros2-nervous-system/06-messages.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (5 tasks)
2. Complete Phase 2: Research & Design (20 tasks) - CRITICAL foundation
3. Complete Phase 3: User Story 1 (10 tasks)
4. **STOP and VALIDATE**: Test US1 independently
   - Can readers answer "What role does ROS 2 play?" after reading Sections 01-02?
   - Are citations complete?
   - Is nervous system analogy clear?
5. Deploy/demo if ready

### Incremental Delivery (Recommended)

1. Complete Setup + Research → Foundation ready (Phases 1-2: 25 tasks)
2. Add User Story 1 (US1) → Sections 01-02 complete → Validate independently (Phase 3: 10 tasks)
3. Add User Story 3 (US3) → Sections 03-06 complete → Validate independently (Phase 4: 23 tasks)
4. Add User Story 4 (US4) → Sections 07-08 complete → Validate independently (Phase 5: 14 tasks)
5. Add User Story 2 (US2) → Section 09 complete → Validate synthesis works (Phase 6: 10 tasks)
6. Add User Story 5 (US5) → Section 10 complete → Validate independently (Phase 7: 10 tasks)
7. Complete Module → Add summary and landing page (Phase 8: 8 tasks)
8. Polish and validate → Final quality checks (Phase 9: 14 tasks)

**Total Tasks**: 114 tasks

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Research together (Phases 1-2)
2. Once Research is done (Phase 2 checkpoint):
   - Creator A: User Story 1 (Sections 01-02)
   - Creator B: User Story 3 (Sections 03-06)
   - Creator C: User Story 4 (Sections 07-08)
   - Creator D: User Story 5 (Section 10)
3. After A, B, C complete:
   - Creator A or B: User Story 2 (Section 09) - synthesizes A, B, C work
4. After all content complete:
   - Creator A: Module summary (Section 11)
   - Creator B: Landing page (index.md)
5. All creators: Final validation (Phase 9) in parallel

---

## Task Summary

- **Total Tasks**: 114
- **Setup Tasks**: 5 (Phase 1)
- **Research & Design Tasks**: 20 (Phase 2)
- **User Story 1 Tasks**: 10 (Phase 3)
- **User Story 3 Tasks**: 23 (Phase 4)
- **User Story 4 Tasks**: 14 (Phase 5)
- **User Story 2 Tasks**: 10 (Phase 6)
- **User Story 5 Tasks**: 10 (Phase 7)
- **Integration Tasks**: 8 (Phase 8)
- **Polish & Validation Tasks**: 14 (Phase 9)

### Tasks by User Story

| User Story | Priority | Sections | Task Count | Can Start After |
|------------|----------|----------|------------|-----------------|
| US1: Understanding ROS 2's Role | P1 | 01-02 | 10 | Phase 2 |
| US3: Learning Core Concepts | P2 | 03-06 | 23 | Phase 2 |
| US4: Python/rclpy Integration | P2 | 07-08 | 14 | Phase 2 |
| US2: Tracing Message Flow | P1 | 09 | 10 | Phase 2 (recommended after US1, US3, US4) |
| US5: URDF Concepts | P3 | 10 | 10 | Phase 2 |

### Parallel Opportunities Identified

- **Phase 1**: 5 parallel tasks (T001-T005)
- **Phase 2 Diagrams**: 9 parallel tasks (T008-T016)
- **Phase 2 Code Examples**: 9 parallel tasks (T017-T025)
- **Phase 3**: 2 parallel section writing tasks (T026-T027)
- **Phase 4**: 4 parallel section writing tasks (T036-T039)
- **Phase 5**: 2 parallel section writing tasks (T059-T060)
- **Phase 9**: 4 parallel validation tasks (T101-T103, T111-T112)

**Estimated Parallel Speedup**: With 4 content creators working in parallel on Phases 3-7, estimated time reduction of ~60%

### Suggested MVP Scope

**MVP = User Story 1 (Sections 01-02)**
- Delivers core "why ROS 2?" understanding
- Uses nervous system analogy
- Independently testable
- **Task Count**: 35 tasks (Setup + Research + US1)
- **Estimated Effort**: ~10-12 hours

### Full Module Scope

**All User Stories (Sections 01-11)**
- **Task Count**: 114 tasks
- **Estimated Effort**: 30-40 hours (per plan.md estimate)

---

## Notes

- **[P] tasks** = Different files, no dependencies - can run in parallel
- **[Story] label** maps task to specific user story for traceability and independent testing
- Each user story delivers sections that are independently completable and testable
- Research phase (Phase 2) is CRITICAL - blocks all content writing
- No automated tests included - specification explicitly states conceptual questions only
- All code examples are illustrative (10-20 lines), not production-ready
- All citations must reference official ROS 2 documentation with retrieval dates
- Commit after each task or logical group for version control
- Stop at any checkpoint to validate story independently before proceeding
