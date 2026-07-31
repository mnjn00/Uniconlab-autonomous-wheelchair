# UNICON Problem Visual Report Design System

## 0. Research Log

- Embedded reference: the user selected `ppt-consulting-precision-grid`; its package tokens and repository-local design constraints are the visual contract.
- Existing project: the report reuses the approved commercial-readiness evidence, 14-slide decision story, and route/band runtime data.
- Skipped Lazyweb and Imagen: this is not greenfield visual direction. The user already approved a concrete style, and the report must visualize measured engineering data rather than introduce an external visual language.
- Design read: an internal engineering decision report for Korean technical and nontechnical teammates, using a precise consulting grid and one blue failure signal.
- Dials: `DESIGN_VARIANCE=3`, `MOTION_INTENSITY=1`, `VISUAL_DENSITY=6`.

## 1. Atmosphere & Identity

The report feels like a calm incident review wall: exact, traceable, and readable in a meeting without technical narration. Its signature is the same measured blue mark appearing on the real route map, the evidence diagrams, and the closure ledger. Blue means “look here and resolve this”; every other visual remains grayscale.

## 2. Color

### Palette

| Role | Token | Value | Usage |
|---|---|---:|---|
| Canvas | `--canvas` | `#FFFFFF` | Page background |
| Surface | `--surface` | `#F4F5F7` | Figure and evidence surfaces |
| Text | `--ink` | `#1A1A1A` | Primary text and diagram lines |
| Muted | `--muted` | `#6B7280` | Secondary labels and sources |
| Accent | `--accent` | `#0B5FFF` | Failure marks, active evidence, focus |
| Border | `--border` | `#D1D5DB` | Hairlines and component boundaries |
| Chart gray 1 | `--chart-gray-1` | `#C9CDD3` | De-emphasized data |
| Chart gray 2 | `--chart-gray-2` | `#9CA3AF` | De-emphasized data |
| Chart gray 3 | `--chart-gray-3` | `#E5E7EB` | De-emphasized data |

### Rules

- Use no second accent, traffic-light colors, gradients, glows, or shadows.
- Pair blue with a label, index, shape, or border so meaning never depends on color alone.
- Large map panels may use dark grayscale pixels from the measured PCD, while every overlay remains from this palette.

## 3. Typography

### Scale

| Level | Size | Weight | Line height | Usage |
|---|---:|---:|---:|---|
| Display | `clamp(2rem, 5vw, 4rem)` | 700 | 1.08 | Page decision |
| H1 | `clamp(1.75rem, 3.5vw, 3rem)` | 700 | 1.16 | Major conclusion |
| H2 | `clamp(1.35rem, 2.3vw, 2rem)` | 700 | 1.24 | Section conclusion |
| H3 | `1.125rem` | 700 | 1.35 | Figure and ledger label |
| Body large | `1.125rem` | 400 | 1.65 | Lead explanation |
| Body | `1rem` | 400 | 1.65 | Default text |
| Small | `0.875rem` | 400 | 1.55 | Secondary evidence |
| Caption | `0.75rem` | 400 | 1.5 | Sources and figure notes |
| KPI | `clamp(2rem, 4vw, 3.5rem)` | 700 | 1 | Measured values |

### Font stack

- Primary: `"Pretendard", "Apple SD Gothic Neo", "NanumSquare Neo", Arial, sans-serif`
- Mono: `"SFMono-Regular", Menlo, Consolas, monospace`

### Rules

- Korean uses `word-break: keep-all`; long paths and hashes use `overflow-wrap: anywhere`.
- Body text never renders below 14px. Source text never renders below 12px.
- Every major heading states the conclusion, not only the topic.

## 4. Spacing & Layout

### Base unit

Spacing uses an 8px base.

| Token | Value | Usage |
|---|---:|---|
| `--space-1` | `0.5rem` | Tight inline spacing |
| `--space-2` | `1rem` | Standard inner spacing |
| `--space-3` | `1.5rem` | Figure and row spacing |
| `--space-4` | `2rem` | Section inner spacing |
| `--space-6` | `3rem` | Major section spacing |
| `--space-8` | `4rem` | Page rhythm |

### Grid

- Maximum content width: 1440px.
- Desktop: 12 columns, 24px gutters.
- Tablet: 6 columns, 20px gutters.
- Mobile: one readable column, 16px page margin.
- Breakpoints: 768px and 1080px.

### Rules

- Page sections use square corners, one-pixel borders, and tonal surfaces.
- Use a single dominant figure per problem. Supporting values sit beside or directly below it.
- Never force primary content into horizontal scrolling.

## 5. Components

### Report shell

- **Structure**: `header + nav + main + footer`.
- **Variants**: full report, print/PDF.
- **Spacing**: `--space-2` through `--space-8`.
- **States**: static.
- **Accessibility**: semantic landmarks and skip link.
- **Motion**: none.
- **Layout**: centered responsive shell.

### Section header

- **Structure**: conclusion heading, one-sentence explanation, hairline.
- **Variants**: page, section.
- **States**: static.
- **Accessibility**: ordered heading levels and Korean plain language.
- **Motion**: none.

### Metric strip

- **Structure**: 2-4 measured values with label and scope note.
- **Variants**: neutral, active.
- **States**: static.
- **Accessibility**: values include units and explanatory text.
- **Motion**: none.
- **Layout**: responsive grid; active metric uses a four-pixel blue left edge and explicit `현재 차단 근거` label.

### Evidence figure

- **Structure**: figure, image or SVG, caption, source.
- **Variants**: route overview, hotspot sheet, process diagram, comparison plot.
- **States**: static.
- **Accessibility**: concise `alt`, visible caption, full text alternative.
- **Motion**: none.
- **Layout**: full-width or 8+4 column split.

### Issue ledger

- **Structure**: priority, problem, observed evidence, effect, closure condition.
- **Variants**: compact, detailed.
- **States**: static.
- **Accessibility**: semantic list or table, no color-only severity.
- **Motion**: none.
- **Layout**: bordered rows on desktop; labeled blocks on mobile.

### Anchor navigation

- **Structure**: native links to major problem sections.
- **Variants**: normal, current focus.
- **States**: default, hover, focus-visible, active.
- **Accessibility**: visible labels, clear focus outline, no icon-only control.
- **Motion**: none.
- **Layout**: wrapping cluster.

### Source note

- **Structure**: source label and local path or observation ID.
- **Variants**: figure, section.
- **States**: static.
- **Accessibility**: path wraps without clipping.
- **Motion**: none.

## 6. Motion & Interaction

- The report is static. No entrance, scroll, hover, parallax, or decorative motion is used.
- Links change border and background immediately on hover/focus; this is state feedback, not animation.
- `prefers-reduced-motion` therefore produces the same surface.

## 7. Depth & Surface

- Strategy: borders plus tonal shift.
- Radius: 0px everywhere.
- Shadows: none.
- Surface hierarchy: white page, gray evidence surface, white plot interior where required.
- A four-pixel blue edge identifies one decision-driving element in a group.

## 8. Accessibility Constraints & Accepted Debt

### Constraints

- WCAG 2.2 AA contrast target.
- All problem meaning survives grayscale through labels, marker shapes, line weight, and numbered hotspots.
- Keyboard users can reach every navigation link and see focus.
- Korean text must not show tofu, clipped glyphs, or orphaned one-syllable lines at 375px, 768px, or 1280px.
- Images have visible captions and nearby text alternatives.
- The printable PDF must keep figures and their captions together.

### Accepted debt

| Item | Location | Why accepted | Owner / Exit |
|---|---|---|---|
| None | - | No accessibility or visual debt is accepted before QA. | - |
