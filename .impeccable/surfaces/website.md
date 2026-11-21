---
version: 1
slug: "website"
primary_target: "website"
related_targets: []
---

# website/ — 上手文档站点

Scope: the whole docs site under `website/`. Visitor mode: Read.

Audience and job: a new student or a colleague taking over a roadshow, with the existing kit and a fresh Linux laptop, following the site from zero to a running demo and then staffing the booth alone. Read beside a powered robot arm, under exhibition-hall light, copying commands as they go.

Content: installation, wiring and power, per-machine config, first run and verification, staffing the show, troubleshooting; a branch for a brand-new kit (first power-up, extrinsic calibration); a quick-reference page. Source of truth is the repository docs; unverified parts are labelled as such.

Constraints: Chinese copy; commands verbatim; no invented photos, numbers or claims; static output deployed to Cloudflare Workers.

Chosen by the user on the decision page: the transit-diagram direction (catalog world `wayfinding-cartography-signage-midnight-transit-diagram`), from the locked sketch `.impeccable/mocks/decision/challenger-transit.html`.

## Direction contract

THESIS: Getting started is a journey on a line. Every page is a station, the brand-new-kit path is a branch that leaves and rejoins the main line, and the reader always sees which station they are at and what the next one is. Refuses the default docs shell of white page, grey sidebar tree and blue links.

OWN-WORLD: Midnight-blue enamel rail carrying porcelain-white station ticks and line inks: scarlet for the main line, amber for the new-kit branch, green for the on-site line, cobalt for reference. Lines bend only at 45 and 90 degrees. Interchange stations are white rings. The reading column is porcelain with deep-ink text. Code sits on enamel. Notices are service advisories. State is a mark: a filled disc for verified on hardware, a hollow ring for not yet verified.

STORY: The reader sees the whole route, knows how far it is, takes the main line if they have the existing kit, changes to the amber branch if the kit is new, and arrives at a running booth.

FIRST VIEWPORT: Left, a fixed enamel rail 380px wide: product name at top, the vertical line with seven stations, the current one largest and lit, the amber branch leaving at 45 degrees. Right, the porcelain column: page title at display size, one lead paragraph, then the first numbered step with its command block. Within the column each step heading carries a station dot on a thin continuation of the page's line.

FORM: transit diagram; the user's pick over the assigned grounded direction (engineering drawing sheet, fourth on the grounded list). seed:6ce7ba641fb93df5

Signature interaction: moving between pages, the lit station marker travels along the line to the next station. Motion grammar: one travel per navigation, exponential ease-out, nothing else moves; reduced motion shows the marker already there.

FINISH: unreviewed and undocumented is unfinished; this build ends with the finish review, the verdict, DESIGN.md, and every shipping raster carrying its provenance
