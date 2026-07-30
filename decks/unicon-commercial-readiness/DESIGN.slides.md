---
version: alpha
name: UNICON Commercial Readiness — Consulting Precision Grid
description: A calm Korean decision deck where evidence is grayscale and one blue signal identifies the decision-driving fact.
derived-from: /Users/minjun/node_modules/slides-grab/src/design-diversity-data.js#L18-L120
medium: slides-16x9
style-id: ppt-consulting-precision-grid
colors:
  canvas: "#FFFFFF"
  surface: "#F4F5F7"
  ink: "#1A1A1A"
  muted: "#6B7280"
  accent: "#0B5FFF"
  border: "#D1D5DB"
  chart-gray-1: "#C9CDD3"
  chart-gray-2: "#9CA3AF"
  chart-gray-3: "#E5E7EB"
---

# UNICON Commercial Readiness Slide Design System

## 0. Research & Evidence Log

- Concrete reference: user-approved bundled style `ppt-consulting-precision-grid`; authoritative local record is `/Users/minjun/node_modules/slides-grab/src/design-diversity-data.js:18-120`.
- Constraints memo: `/Users/minjun/unicon-wheelchair/.omo/teams/team-461f02b1/artifacts/consulting-precision-grid-constraints.md`.
- Approved content: `/Users/minjun/unicon-wheelchair/decks/unicon-commercial-readiness/slide-outline.md`; approval receipt records SHA-256 `805f7073d84fcbbad454690322a892567520cce0034b686454a2bb8d113b44a3`.
- External brand, lazyweb, and Imagen lanes were not run: the user supplied an exact local style contract and explicitly required that it win over generic frontend taste.
- Interaction research was not run: the output is a static slide deck with no controls, hover, loading, or application state.

## 1. Atmosphere & Identity

### Overview

This deck is a quiet evidence room: exact, candid, and operational. It should feel like a high-stakes consulting exhibit prepared for a mixed executive, engineering, safety, and quality audience. The signature is a complete Korean conclusion sentence above a 1px hairline, followed by a precisely aligned grayscale exhibit with one—and only one—blue decision signal.

### Background

- Primary canvas: solid `--color-canvas` only.
- Evidence surfaces: solid `--color-surface` used to group data, never to imitate cards for decoration.
- No dark mode, full-bleed photography, gradients, textures, glows, or decorative background shapes.

### Design principles

1. **Answer first:** the title states the decision or evidence conclusion, not the topic.
2. **Evidence remains inspectable:** sources have a fixed bottom-right zone and data-bearing slides never omit them.
3. **One blue target:** blue identifies the decision-driving datum, edge, KPI, or verdict; all other information remains grayscale.
4. **No color-only meaning:** labels, border weight, position, and badges carry state when color is unavailable.
5. **Split, never shrink:** dense Korean content moves to another slide before body copy drops below 16pt or sources below 10pt.

## 2. Color

### Colors

| Role | CSS token | Value | Usage |
|---|---|---:|---|
| Canvas | `--color-canvas` | `#FFFFFF` | Every slide background |
| Surface | `--color-surface` | `#F4F5F7` | Evidence modules, nodes, ledger headers |
| Ink | `--color-ink` | `#1A1A1A` | Titles, body, connector strokes |
| Muted ink | `--color-muted` | `#6B7280` | Kicker, metadata, source caption |
| Accent | `--color-accent` | `#0B5FFF` | One decision-driving target per slide |
| Border | `--color-border` | `#D1D5DB` | Hairlines, neutral module borders |
| Chart gray 1 | `--color-chart-1` | `#C9CDD3` | De-emphasized chart series |
| Chart gray 2 | `--color-chart-2` | `#9CA3AF` | De-emphasized chart series |
| Chart gray 3 | `--color-chart-3` | `#E5E7EB` | De-emphasized chart series |

### Color rules

- Raw palette values exist only in the global token layer. Components consume semantic or component tokens.
- A slide may expose one accent target. Repeating blue borders across every module is a contract violation.
- Running body text is ink or muted ink, never accent.
- Pass, caution, fail, unknown, NO-GO, and GO are written as plain-language labels; no red, amber, or green status palette is introduced.
- Every accent use also has a non-color cue: label, 4px edge, bold weight, numbered badge, or positional emphasis.

## 3. Typography

### Font stack

- Shared slide stack: `Arial, Helvetica, "Liberation Sans", "Pretendard", "Apple SD Gothic Neo", "Noto Sans KR", "Malgun Gothic", sans-serif`.
- Latin labels and figures resolve to Arial first; Korean glyphs fall through to Pretendard as the first CJK-capable face.
- If Pretendard is unavailable, the next Korean sans fallback is a verification limitation, not permission to change metrics.

### Scale

| Role | CSS token | Size | Weight | Line height | Tracking | Usage |
|---|---|---:|---:|---:|---:|---|
| Action title | `--type-title-*` | 24pt | 700 | 1.15 | -0.01em | One complete conclusion sentence |
| Kicker | `--type-kicker-*` | 11pt | 700 | 1.2 | 0.08em | Section/primitive label |
| Body | `--type-body-*` | 16pt | 400 | 1.3 | 0 | Explanations and evidence |
| KPI | `--type-kpi-*` | 40pt | 700 | 1.2 | -0.02em | One dominant value |
| Source | `--type-source-*` | 10pt | 400 | 1.2 | 0 | Fixed source caption |

### Typography rules

- Apply `word-break: keep-all` and `overflow-wrap: break-word` to Korean text.
- Body copy is left-aligned and limited to 4–5 readable lines; six is the absolute contract ceiling.
- Avoid orphaned Korean particles, endings, parentheticals, and broken technical terms.
- No text renders below 10pt. Never reduce type to fit a dense table or diagram.
- Use semantic headings and paragraphs; visual size never substitutes for heading hierarchy.

## 4. Spacing & Slide Layouts

### Frame and grid

- Fixed slide surface: 720pt × 405pt, 16:9.
- Grid: 12 equal columns.
- Proportional margins: 0.6in horizontal and 0.5in vertical relative to the 13.33 × 7.5in source contract.
- Gutter: 0.15in proportional to the source contract.
- Spacing base: 8px intent, translated into named CSS tokens.
- Header band: 0.5–1.4in source range, implemented as a fixed header zone sized for one 24pt Korean sentence.
- Source zone: fixed bottom-right and reserved before placing content.
- Maximum visual modules: three per slide.

### Slide Layouts

- **Cover**
  - One action-title verdict and a two-card decision pair.
  - No navigation, CTA, icon, or illustration.
  - Exactly one card may be the blue target.
- **Section divider**
  - Short kicker plus one answer-first title.
  - Optional single evidence line; never a dashboard.
- **Content**
  - One dominant exhibit with one or two support modules.
  - Use 60/40 or 50/50 grid spans; do not exceed three modules.
- **Statistic**
  - One KPI/value primitive plus one short explanation.
  - Other figures remain body-scale grayscale evidence.
- **Ledger**
  - Three to five rows with explicit state labels.
  - One selected row may carry the accent edge; no zebra colors.
- **Two-lane diagram**
  - Two grayscale lanes with square nodes and straight connectors.
  - One active edge/node identifies the evidence break.
- **Decision**
  - Paired NO-GO/GO or current/next cards distinguished by visible labels.
  - Color is secondary to label and border weight.
- **Gate timeline**
  - Gate 0–5 nodes in one reading order.
  - One current-next edge/label is blue; a bracket states scope separately from authority.
- **Closing**
  - Restates the decision pair and review trigger.
  - Footer/source strip remains one line; no multi-column footer.

## 5. Primitives & Static States

Text, evidence, KPI, decision, caveat, and ledger primitives use live semantic DOM, CSS tokens, and reusable classes. Multi-node authority chains and Gate 0–5 flows use local tldraw-generated SVG assets plus an adjacent semantic transcript. This is a static deck, so hover, focus, active, disabled, loading, success, error, and empty application states are **N/A unless a state is content being reported**. Do not invent interactive styling.

### Slide frame, header, hairline, and source

- **Structure:** `<section class="slide-frame">` containing `<header>`, `<h1>`, hairline `<div aria-hidden="true">`, a content `<div>`, and `<footer class="slide-source">`; the document itself owns the single semantic `<main>`.
- **Variants:** standard frame; cover and closing are composition recipes for later slide authoring, not separate Stage 2 component states.
- **Tokens:** frame size, margins, header zone, canvas, ink, border, source type.
- **Static states:** default only. Hover/focus/loading/error are N/A.
- **Accessibility:** one H1 per frame; source text remains selectable and at least 10pt.

### Evidence module

- **Structure:** semantic `<article>` with kicker, heading, and one short paragraph/list.
- **Variants:** neutral surface and selected target are rendered; compact is a spacing recipe with no separate state behavior.
- **Tokens:** surface, border, accent-edge, body scale, module padding.
- **Static states:** neutral and selected. Selected combines label + 4px edge; no color-only state.
- **Accessibility:** no more than one heading level inside the module; long copy splits to another slide.

### KPI/value

- **Structure:** `<article>` with visible label, `<strong>` value, and one-sentence interpretation.
- **Variants:** neutral; selected target.
- **Tokens:** KPI scale, body scale, ink/accent.
- **Static states:** neutral and selected. No animated count-up or loading skeleton.
- **Accessibility:** value meaning is written in adjacent text; color does not carry direction.

### Ledger

- **Structure:** semantic `<table>` with `<caption>`, scoped headers, and up to five body rows.
- **Variants:** the evidence ledger is rendered; a closure ledger reuses the same table semantics and tokens with closing-slide content.
- **Tokens:** hairline, surface header, ink, muted, selected-row edge.
- **Static states:** confirmed, unknown, pass condition; represented with plain-language row labels. No interactive sorting/loading.
- **Accessibility:** column headers use `scope="col"`; row labels use `scope="row"`; a short summary precedes complex evidence.

### Two-lane diagram

- **Structure:** local `authority-chain.svg` generated from `authority-chain.tldr`, with an adjacent visually hidden ordered-list transcript.
- **Variants:** software vs field; expected vs observed.
- **Tokens:** lane gap, node size, connector stroke, neutral surface, accent edge.
- **Static states:** neutral lane; one active break/edge. Hover/focus/loading are N/A.
- **Accessibility:** each lane has a visible name in the asset, a semantic transcript in DOM order, descriptive alternative text, and a plain-language summary after the diagram.

### Square node and connector

- **Structure:** square-corner tldraw node with visible number/state label; connector is a straight 0.75pt rule with a 4px triangular arrowhead.
- **Variants:** neutral; active; unknown.
- **Tokens:** 1px neutral border, 4px active left edge, 0.75pt connector stroke, node surface.
- **Static states:** state is content, never interaction. Active/unknown must include text.
- **Accessibility:** reading order follows DOM order; connector direction is repeated in text.

### Decision card

- **Structure:** `<article>` with target label, decision heading, and one concise consequence.
- **Variants:** NO-GO; GO program scope; neutral.
- **Tokens:** surface/canvas, border, selected edge, title/body scale.
- **Static states:** neutral and selected target. Hover/focus/disabled/loading are N/A.
- **Accessibility:** the card starts with the 대상 label, so the decision remains clear in grayscale.

### Caveat strip

- **Structure:** `<aside>` with a visible caveat label and one plain-language sentence.
- **Variants:** interpretation boundary; unknown-not-absent; scope boundary.
- **Tokens:** hairline, surface, muted/ink, kicker/body scale.
- **Static states:** default only.
- **Accessibility:** never relies on symbol-only warning language or a colored alert icon.

### Gate timeline

- **Structure:** local `gate-timeline.svg` generated from `gate-timeline.tldr`, an adjacent ordered-list transcript, and a separate live-DOM scope bracket/label.
- **Variants:** current; next required; future; scope-only.
- **Tokens:** node, connector, badge, accent edge, muted label.
- **Static states:** one next-required edge/label may be selected; future gates remain grayscale.
- **Accessibility:** each gate includes its number, name, and authority condition in the semantic transcript; sequence is usable without the image.

## 6. Motion & Interaction

- Static slide primitives have no interactive controls, hover affordances, loading indicators, animated counters, or decorative motion.
- Slide-level transition may be cut or a restrained 0.2s fade in the eventual viewer; it does not alter content or reading order.
- `prefers-reduced-motion: reduce` removes the optional viewer fade.
- Keyboard, focus, pointer, touch, pressed, and recovery states are **N/A for the primitive showcase** because it contains no interactive element.
- If an editor/viewer later adds controls, those controls belong to the viewer design system and must not be styled as slide content.

## 7. Depth, Surface, Signature Motifs & Avoid

### Depth strategy

Use a **mixed tonal-and-rule hierarchy**: white canvas, light-gray evidence surfaces, hairline borders, scale contrast, alignment, and whitespace. No shadows, gradients, blur, glow, bevel, or pseudo-3D treatment.

### Signature motifs

1. Complete action-title sentence over a 1px hairline.
2. Square evidence modules aligned to a strict 12-column grid.
3. One 4px blue edge or blue KPI per slide.
4. Circular sequence badges used only for functional diagram order.
5. Fixed bottom-right source caption.

### Avoid

- Top navigation, sticky headers, menu rows, CTA buttons, pricing grids, and web-style multi-column footer bands.
- Hover, focus, pressed, disabled, loading, success, or error styling inside slide content.
- Rounded cards, shadows, gradients, glass, glows, decoration-first shapes, icons, emoji, clipart, and stock imagery.
- Multiple accent colors, colored running text, red/green decision semantics, rainbow charts, legend boxes, 3D charts, truncated axes, or `div`-built fake bars.
- Generic 3×2 feature-card grids, repeated accent-left cards, and dense mini-dashboards.
- More than three visual modules, more than five table rows, body below 16pt, source below 10pt, or six-plus prose lines.

## 8. Accessibility Constraints, Inclusive Personas & Open Debt

### Inclusive personas

#### Nontechnical Korean decision maker

- **Goal:** identify the release decision, evidence boundary, and next authority step in seconds.
- **Pass:** one answer-first sentence, 2–4 concise details, no unexplained jargon, no more than three modules.
- **Fail:** topic-label titles, acronym clusters, or a long table that requires working-memory reconstruction.

#### Safety engineer

- **Goal:** trace each claim to a source and distinguish confirmed, unknown, blocked, and pass-condition evidence.
- **Pass:** visible source caption, exact labels, explicit authority conditions, and consistent node/ledger semantics.
- **Fail:** missing provenance, decorative state colors, or diagrams whose edge meaning exists only visually.

#### Low-vision / 200% zoom reader

- **Goal:** read Korean text and decision labels without text loss or color dependence.
- **Pass:** 16pt body, 10pt minimum source, Pretendard as the first CJK-capable fallback, non-color state cues, and no clipped content when the fixed slide is magnified and panned.
- **Fail:** type reduction, tofu glyphs, color-only selection, clipped source captions, or hidden overflow.

### Adaptive and cognitive constraints

- Keep one primary decision per frame and chunk supporting evidence into 2–4 concise details.
- Use plain Korean labels such as `확인됨`, `알 수 없음`, `통과 조건`, `현재`, and `다음 필수`.
- Preserve a consistent title → exhibit → caveat/source reading order.
- `word-break: keep-all` is mandatory; inspect every Korean line break in browser captures.
- At 200% zoom, this fixed two-dimensional slide canvas may require panning, but no slide content may disappear, overlap, or be truncated; overflowing showcase frames align to a reachable start edge.
- High-contrast and grayscale inspection must preserve state meaning through labels, weight, edge thickness, and order.

### Static state declaration

- The deck is static. Hover, focus, active, disabled, loading, success, error, and application-empty states are N/A in Sections 5 and 8.
- Reported evidence states such as confirmed, unknown, NO-GO, and next-required are content variants, not interactive states.
- Do not fabricate controls or motion to satisfy a web component-state checklist.

### Open debt / known limitations

| ID | Date | Source | Item | Location | Affected persona | Acknowledged | Notes / exit |
|---|---|---|---|---|---|---|---|
| DS-001 | 2026-07-30 | local `fc-match` and font search | Pretendard binary is not installed in the current runtime; Korean glyphs may use Apple SD Gothic Neo. | `assets/deck.css`, browser evidence | Low-vision/CJK reader | No | Open. Before final deck approval, vendor a licensed local Pretendard webfont or install it in the render runtime, then recapture and confirm glyph metrics. |
| DS-002 | 2026-07-30 | fixed-slide format constraint | Responsive reflow is not applied inside the fixed 720pt × 405pt canvas. | All frames | 200% zoom reader | No | Open constraint. Pan remains reachable from the start edge; a final viewer may add an accessible transcript/reflow view. |

### Source mapping

- Bundled action title + hairline → slide header primitive.
- Bundled 12-column grayscale data boxes → evidence module, ledger, and decision-card primitives.
- Bundled one-blue-target rule → selected target variant with a label and 4px edge.
- Bundled square nodes + straight connectors + numbered badges → two-lane and Gate timeline primitives.
- Bundled fixed source caption → reserved bottom-right source primitive at 10pt.
- Web navigation, CTA, hover, loading, footer menus, and atmospheric effects → dropped as inapplicable to static slides.
