---
name: SO-ARM 上手线路
description: website/ 文档站点的视觉系统：珐琅深蓝的线路牌、瓷白的阅读栏、四条线的颜色。不适用于 frontend/ 控制台。
colors:
  scarlet: "#e03a2f"
  scarlet-ink: "#bf271d"
  amber: "#f2a900"
  amber-ink: "#7a5300"
  green: "#1c9a57"
  green-ink: "#0f6e3c"
  cobalt: "#2c6be0"
  cobalt-ink: "#1d52bd"
  enamel: "#0e1a3c"
  enamel-up: "#19295a"
  enamel-hover: "#24387a"
  on-enamel: "#f4f6fa"
  on-enamel-2: "#aab8dc"
  porcelain: "#f4f6fa"
  paper: "#ffffff"
  code-tint: "#e4e9f3"
  ink: "#121a30"
  ink-2: "#465070"
  rule: "#d3d9e7"
typography:
  display:
    fontFamily: "Overpass Variable, Noto Sans SC Variable, PingFang SC, Microsoft YaHei, Noto Sans CJK SC, sans-serif"
    fontSize: "clamp(2.25rem, 1.6rem + 2.6vw, 3.25rem)"
    fontWeight: 800
    lineHeight: 1.15
    letterSpacing: "-0.01em"
  headline:
    fontFamily: "Overpass Variable, Noto Sans SC Variable, PingFang SC, Microsoft YaHei, Noto Sans CJK SC, sans-serif"
    fontSize: "1.625rem"
    fontWeight: 800
    lineHeight: 1.3
  title:
    fontFamily: "Overpass Variable, Noto Sans SC Variable, PingFang SC, Microsoft YaHei, Noto Sans CJK SC, sans-serif"
    fontSize: "1.1875rem"
    fontWeight: 700
    lineHeight: 1.4
  lead:
    fontFamily: "Overpass Variable, Noto Sans SC Variable, PingFang SC, Microsoft YaHei, Noto Sans CJK SC, sans-serif"
    fontSize: "1.25rem"
    fontWeight: 400
    lineHeight: 1.7
  body:
    fontFamily: "Overpass Variable, Noto Sans SC Variable, PingFang SC, Microsoft YaHei, Noto Sans CJK SC, sans-serif"
    fontSize: "1.0625rem"
    fontWeight: 400
    lineHeight: 1.8
    fontFeature: "tnum"
  label:
    fontFamily: "Overpass Variable, Noto Sans SC Variable, PingFang SC, Microsoft YaHei, Noto Sans CJK SC, sans-serif"
    fontSize: "0.9375rem"
    fontWeight: 700
    lineHeight: 1.4
  label-small:
    fontFamily: "Overpass Variable, Noto Sans SC Variable, PingFang SC, Microsoft YaHei, Noto Sans CJK SC, sans-serif"
    fontSize: "0.8125rem"
    fontWeight: 700
    lineHeight: 1.45
  mono:
    fontFamily: "JetBrains Mono Variable, ui-monospace, SFMono-Regular, Menlo, Consolas, monospace"
    fontSize: "0.9375rem"
    fontWeight: 400
    lineHeight: 1.75
    fontFeature: "zero"
rounded:
  sm: "4px"
  md: "8px"
  full: "999px"
spacing:
  paragraph: "1.15rem"
  block: "1.75rem"
  subsection: "2.5rem"
  section: "4rem"
  page-top: "5.5rem"
  measure: "44rem"
  side: "380px"
components:
  rail:
    backgroundColor: "{colors.enamel}"
    textColor: "{colors.on-enamel-2}"
    width: "{spacing.side}"
    padding: "0.25rem 2.25rem 1.5rem"
  rail-stop-here:
    textColor: "{colors.on-enamel}"
  menu-button:
    backgroundColor: "{colors.enamel-up}"
    textColor: "{colors.on-enamel}"
    rounded: "{rounded.full}"
    height: "2.75rem"
    padding: "0 0.6rem 0 1rem"
  code-block:
    backgroundColor: "{colors.enamel}"
    textColor: "{colors.on-enamel}"
    typography: "{typography.mono}"
    rounded: "{rounded.md}"
    padding: "1.1rem 1.25rem"
  copy-button:
    backgroundColor: "{colors.enamel-up}"
    textColor: "{colors.on-enamel-2}"
    padding: "0.5rem 0.7rem"
  copy-button-hover:
    backgroundColor: "{colors.enamel-hover}"
    textColor: "{colors.on-enamel}"
  inline-code:
    backgroundColor: "{colors.code-tint}"
    textColor: "{colors.ink}"
    rounded: "{rounded.sm}"
    padding: "0.12em 0.4em"
  notice:
    backgroundColor: "{colors.paper}"
    textColor: "{colors.ink}"
    rounded: "{rounded.md}"
    padding: "0.9rem 1.1rem 1rem"
  notice-label-info:
    backgroundColor: "{colors.cobalt-ink}"
    textColor: "{colors.paper}"
    typography: "{typography.label}"
    padding: "0.5rem 1.1rem"
  notice-label-moves:
    backgroundColor: "{colors.scarlet-ink}"
    textColor: "{colors.paper}"
  notice-label-caution:
    backgroundColor: "{colors.amber}"
    textColor: "{colors.ink}"
  notice-label-unverified:
    backgroundColor: "{colors.paper}"
    textColor: "{colors.ink}"
  onward-link:
    textColor: "{colors.ink}"
    padding: "0.25rem 0"
---

# Design System: SO-ARM 上手线路

> 适用范围：本文件只描述 `website/` 文档站点。仓库里的 `frontend/` 是另一套早已存在的深色操作控制台（无构建步骤），是不同的界面，不要把这套系统套到它身上。

## Overview

**Creative North Star: "夜间线路图"**

上手是一趟沿着线路走的行程：每一页是一站，新套机的流程是一条离开主线又回来的支线，读者随时看得见自己在哪一站、下一站是什么。左边是一块珐琅深蓝的线路牌，承载瓷白的站点和四条线的颜色；右边是瓷白的阅读栏，深墨色的字，命令落在与线路牌同一种深蓝上。

整套系统是平的、密度适中的、为强光下边看边敲命令而设：正文字号偏大（1.0625rem / 行高 1.8），代码块对比强烈，状态靠形状而不是只靠颜色区分。它拒绝的是默认的文档外壳：白页面、灰色目录树、蓝色链接列表。

落地时与方向稿有一处出入：方向稿写“除站点标记外什么都不动”，实际构建保留了 0.2s 的悬停反馈（站名变亮、站点放大到 1.2 倍、复制按钮换底色）和小屏菜单箭头的 0.25s 翻转。以构建为准。

**Key Characteristics:**
- 固定在左侧、380px 宽的珐琅深蓝线路牌，线路只在 45° 和 90° 转弯。
- 四条线四种颜色：主线猩红、新套机支线琥珀、现场线绿、速查线钴蓝；页面通过 `data-line` 继承自己所在线路的颜色。
- 瓷白阅读栏，最宽 44rem，步骤页的每个二级标题是一个带编号的站点圆环。
- 代码落在珐琅深蓝上；提示是带色条标签的“运营通告”；表格是只有横线的时刻表。
- 没有阴影、没有渐变；圆环和分隔线承担全部层次。
- 换页时，亮着的站点标记沿线路移到下一站，这是唯一的大动作。

## Colors

深蓝珐琅与瓷白构成底色，四种线路色只在需要指路或示警的地方出现。

### Primary
- **主线猩红** (`scarlet`)：主线的线条、当前站的实心圆、步骤圆环的边、上一站/下一站上方的粗线，以及示意图里的腕部相机。是默认的 `--line`。
- **猩红墨色** (`scarlet-ink`)：猩红在瓷白底上的文字版本，用于位置行的线路名、列表记号、小屏的步骤编号，以及“机械臂会动”通告的标签底色（配白字）。

### Secondary
- **支线琥珀** (`amber`)：新套机支线的线条和圆环；线路牌上换乘站下方的小字说明；“注意”通告的标签底色（配深墨字）。
- **琥珀墨色** (`amber-ink`)：支线页面在瓷白底上的文字色。

### Tertiary
- **现场线绿** (`green`) / **绿墨色** (`green-ink`)：路演值守、排障两站的线色与对应文字色。
- **速查钴蓝** (`cobalt`) / **钴蓝墨色** (`cobalt-ink`)：速查线的线色；同时是全站的焦点环（3px `cobalt`）、正文链接（`cobalt-ink`）、“说明”通告的标签底色，以及示意图里的深度相机。

### Neutral
- **珐琅深蓝** (`enamel`)：线路牌、代码块、文字选区的底色。
- **珐琅浅一档** (`enamel-up`)：深蓝上的次级表面：复制按钮、小屏菜单按钮、图例上方的分隔线、滚动条。
- **珐琅悬停** (`enamel-hover`)：复制按钮悬停时的底色。源码里是字面量，没有自定义属性。
- **珐琅上的白** (`on-enamel`) / **珐琅上的灰蓝** (`on-enamel-2`)：深蓝上的主文字与次文字；未选中的站名用灰蓝，当前站用白。
- **瓷白** (`porcelain`)：阅读栏的页面底色，也是步骤圆环和线路表圆环的内芯。
- **纸白** (`paper`)：通告卡片和按键帽的底色，比页面亮一档。
- **代码淡底** (`code-tint`)：行内代码的底色。源码里是字面量，没有自定义属性。
- **深墨** (`ink`) / **次墨** (`ink-2`)：正文与次要文字（导语、表头、图注、说明）。
- **分隔线** (`rule`)：表格行线、通告边框、线路表行线。

### Named Rules
**The Page Line Rule.** 每一页通过 `<html data-line>` 设定 `--line` 和 `--line-ink`；步骤圆环、步骤连线、位置行色条、列表记号、页尾粗线都读这两个变量，不写死某一种线色。新页面属于哪条线，就自动是哪种颜色。

**The Ink Pair Rule.** 每种线色都有一个更深的 `-ink` 版本。在瓷白或纸白底上，亮色只画线、圆环和色块，文字一律用 `-ink`；白字只压在 `scarlet-ink` 和 `cobalt-ink` 上，琥珀底上压深墨字。在珐琅深蓝上，亮色可以直接作文字（如换乘站的琥珀小字）。

## Typography

**Display Font:** Overpass Variable（中文回退到 Noto Sans SC Variable，再到 PingFang SC / Microsoft YaHei）
**Body Font:** 同一族，标题与正文共用
**Label/Mono Font:** JetBrains Mono Variable（回退到 ui-monospace、Menlo、Consolas）

**Character:** Overpass 源自公路标识字体，与线路牌同出一脉；Noto Sans SC 接住中文。全站只有一个无衬线族，层级靠字重（400 / 700 / 800）和字号拉开，等宽字只属于代码和按键。

### Hierarchy
- **Display**（800，`clamp(2.25rem, 1.6rem + 2.6vw, 3.25rem)`，行高 1.15，字距 -0.01em，`text-wrap: balance`）：每页唯一的一级标题。
- **Headline**（800，1.625rem，行高 1.3；62rem 以下为 1.4375rem）：二级标题，在步骤页上就是一站。页尾“上一站/下一站”的站名用同一字重（1.375rem）。
- **Title**（700，1.1875rem，行高 1.4）：三级标题。
- **Lead**（400，1.25rem，行高 1.7，次墨色）：标题下的一段导语。
- **Body**（400，1.0625rem，行高 1.8，表格数字等宽 `tnum`）：正文，栏宽上限 44rem。表格、图注、代码块内文字降到 0.9375rem。
- **Label**（700，0.9375rem）：位置行、通告标签、小屏菜单按钮。更小的一档是 0.8125rem，用于表头、图例和复制按钮。
- **Mono**（400，代码块内 0.9375rem / 行高 1.75；行内为所在文字的 0.875em；开启带斜线的零 `zero`）：命令、配置项、标识符。

### Named Rules
**The One Family Rule.** 标题、正文、标签同属一个无衬线族；等宽字只出现在代码、命令和按键帽里，不用来做装饰性的标签。

## Layout

两栏结构。左栏是固定定位的线路牌，宽 380px（`--side`），占满视口高度，内容过长时自身滚动。右栏 `main` 让出 380px，内边距为上 5.5rem、左右 `clamp(1.5rem, 6vw, 6.5rem)`、下 5rem；正文和页尾导航的最大宽度都是 44rem，靠左对齐，不居中。

纵向节奏：段落间 1.15rem；代码块、表格、通告、图前后约 1.5–2rem；三级标题上方 2.5rem；二级标题上方 4rem；位置行到正文 3.5rem；页尾导航上方 5rem。锚点滚动预留 5rem。

步骤页（默认；总览、排障、速查三页关闭）在 62rem 以上把步骤编号挂在正文左侧的留白里：圆环中心距正文左缘约 3rem，一条 6px 宽、30% 不透明度的线从第一个二级标题向下贯穿到文末。线的起点由脚本量出；没有脚本时只有圆环，不画线。

62rem 以下：线路牌收成吸顶的顶栏，只留品牌名和一个胶囊形菜单按钮，按钮上写着当前站名，点开后展开整张线路图；`main` 的内边距变为 2.5rem 1.25rem 3.5rem；步骤编号改为标题前的行内数字，用本页的 `--line-ink`；线路表从三列收成两列；图例从两列收成一列；示意图里的标注隐藏，改用图下方的图例列表。

打印时隐藏线路牌、页尾导航和复制按钮，代码块改为白底黑框。

## Elevation & Depth

完全平面。全站没有投影，也没有渐变。层次来自三样东西：深蓝与瓷白两块底色的对比、纸白卡片比瓷白页面亮一档、以及 1px 到 6px 的线。

`box-shadow` 只以零模糊的实心环出现，用来把站点从线路上“切”出来：普通站点外圈 3px 珐琅色；换乘站和当前站是 4px 白环外加 7px 珐琅色。这是线路图的画法，不是阴影。

### Named Rules
**The Solid Ring Rule.** `box-shadow` 只允许 `0 0 0 Npx` 的实心环，用于站点圆点。任何带模糊或偏移的投影都不属于这个系统。

## Shapes

三种形状构成全部语汇：圆、胶囊、小圆角矩形。

- **圆**：站点（18px）、步骤圆环（2.6rem，5px 线色边）、线路表圆环（1.1rem，4px 边）、验证记号（0.8rem，2.5px 边）。
- **胶囊**（999px）：线路牌上的站点链接、小屏菜单按钮；线路色条是两端圆头的短线（宽 1.75rem、高 6px）。
- **小圆角矩形**：代码块和通告 8px；行内代码 4px。复制按钮（6px）和按键帽（5px）各自略有出入，属于单件取值。

线路本身：线宽 8px，圆头圆角连接，只在 45° 和 90° 转弯。主线在 x=34，支线在 x=150，普通站距 52px；支线以 45° 斜线离开主线，再以 45° 斜线回来。

分隔一律用线不用框：表格只有横线，线路表只有行线，页尾导航上方是一条 6px 的本页线色粗线。

### Named Rules
**The 45/90 Rule.** 线路只在 45° 和 90° 转弯，站点只落在线上。新增线路或站点时延续这套几何，不画曲线，不画任意角度。

**The Shape Carries State Rule.** 状态由形状表达，颜色只是加强：已验证是实心圆，未验证是空心环；换乘站是白环；当前站是更大的实心圆加粗体站名。

## Components

### 线路牌导航（Rail）
全站的导航，也是这套系统的签名。一条竖直的线路串起各站，数据来自 `route.ts`，导航、页尾的上一站/下一站、首页线路表都由同一份数据生成。
- **站点：** 18px 白色圆点，外圈 3px 珐琅色环；站名 1rem，灰蓝色。
- **悬停 / 焦点：** 站名变白，圆点放大到 1.2 倍（0.2s）。
- **换乘站：** 珐琅色内芯的白环；站名下有一行琥珀色小字说明换乘关系。
- **当前站：** 圆点填上所在线路的颜色并放大到 1.25 倍，站名变白、加粗、1.125rem，带 `aria-current="page"`。
- **图例：** 线路图下方，一条分隔线之后是两列的图例，每条线一个色条、线名（700）和一句“谁走这条线”。
- **小屏：** 收进顶栏的菜单按钮（胶囊形、最小高度 2.75rem、写着当前站名、箭头展开时翻转 180°）。
- **换页动效：** 当前站圆点带 `view-transition-name`，跨页导航时用 0.55s、`cubic-bezier(0.16, 1, 0.3, 1)` 从上一站移到下一站；页面其余部分 0.18s 淡入淡出。系统设置为减少动态效果时全部关闭，标记直接出现在新位置。

### 位置行
导语下方的一行：一个本页线色的色条、加粗的线路名（`--line-ink`）、以及次墨色的“全程第 N 站，共 M 站”。速查线只显示线名。

### 步骤标题
步骤页的二级标题自动编号。宽屏是挂在左侧的圆环：瓷白内芯、5px 本页线色的边、800 字重的数字；窄屏是标题前的行内数字。

### 代码块
- **样式：** 珐琅深蓝底、8px 圆角、等宽字 0.9375rem / 行高 1.75，过长时横向滚动，可用键盘聚焦。
- **语法着色：** 注释 `#8e9cc4`、关键字 `#ffb829`、字符串 `#8fe0b0`、常量与链接 `#9cc0ff`、标点 `#aab8dc`，其余为 `on-enamel`。
- **复制按钮：** 由脚本加在代码块右上角，`enamel-up` 底、灰蓝字、0.8125rem；悬停时底色变亮、字变白；点击后文字依次为“已复制”或“复制失败，请手动选择”，1.6 秒后复原。
- **行内代码：** 淡蓝灰底、4px 圆角、0.875em。表格单元格里的代码不换行。
- **按键帽：** 纸白底、1px 深墨边、底边加厚到 3px、等宽粗体。

### 通告（Notice）
运营通告式的提示。纸白卡片、1px 分隔线色的边、8px 圆角；顶部是一条满宽的标签条（图标 + 700 字重的标题），下面是正文。四种：
- **说明（info）：** 钴蓝墨色标签、白字、圆圈图标。
- **机械臂会动（moves）：** 猩红墨色标签、白字、三角图标。凡是会让机械臂动起来的步骤，都先放这一条。
- **注意（caution）：** 琥珀色标签、深墨字、三角图标。
- **尚未在真机验证（unverified）：** 纸白标签、深墨字、底部一条虚线、空心环图标。刻意不用颜色。

图标是 18px 的内联描边 SVG（线宽 1.8，圆头）。

### 验证记号（Mark）
行内的状态记号，多用在表格里：0.8rem 的圆加一句文字。实心深墨圆是“已在真机验证”，次墨色空心环是“尚未在真机验证”。

### 时刻表式表格
只有横线：表头下方 2px 深墨线，行间 1px 分隔线，末行下方回到深墨线。表头 0.8125rem、700、次墨色；首列加粗；没有底色、没有竖线、没有斑马纹。脚本把表格包进可横向滚动、可聚焦的区域。

### 线路表（RouteTable）
首页用来列出一条线上的各站。每行是一个整行可点的链接：线色圆环、加粗的站名、次墨色的一句说明，行间只有 1px 分隔线。悬停时站名加 2px 下划线。

### 上一站 / 下一站
页尾的两列导航，上方是一条 6px 的本页线色粗线。每侧是一行小字（“上一站”/“下一站”）加 800 字重的站名，下一站靠右对齐。悬停时站名加 3px 的线色下划线。支线两端接回主线的换乘站。

### 示意图（KitFigure）
内联 SVG 的俯视示意图，颜色取自同一套色板：深墨画机械臂，钴蓝是深度相机及其视野（13% 不透明度的色块），猩红是腕部相机及其视野（18%）。标注用细引线接到图形上；窄屏放大图形、隐藏标注，改用图下方的图例。图注为次墨色 0.9375rem。

### 链接与焦点
正文链接为钴蓝墨色，1px 下划线、偏移 0.22em，悬停时加粗到 2px。所有可聚焦元素的焦点环是 3px 钴蓝、偏移 3px。页面最前有“跳到正文”的跳转链接。

## Do's and Don'ts

### Do:
- **Do** 让新页面通过 `route.ts` 登记为某条线上的一站，并由 `data-line` 取得 `--line` / `--line-ink`；页面内的线色元素都读这两个变量。
- **Do** 把所有命令放进珐琅深蓝的代码块，保留复制按钮和横向滚动，命令原文不改写。
- **Do** 在瓷白或纸白底上用 `-ink` 版本的线色写字，亮色只用于线、圆环和色块。
- **Do** 用形状表达状态：实心圆是已验证，空心环是未验证，白环是换乘站。
- **Do** 在任何会让机械臂动起来的步骤之前放一条“机械臂会动”通告。
- **Do** 用线分隔而不是用框：表格只画横线，列表只画行线。
- **Do** 保留 3px 钴蓝焦点环，并在系统要求减少动态效果时关闭全部过渡。
- **Do** 需要画面时用与实物一致的示意图，颜色取自色板。

### Don't:
- **Don't** 把这套系统用到 `frontend/` 的操作控制台上，那是另一个界面。
- **Don't** 使用带模糊或偏移的投影，也不要使用渐变；`box-shadow` 只用于站点的实心环。
- **Don't** 让线路以 45° 和 90° 之外的角度转弯，也不要画曲线。
- **Don't** 在同一张线路图上点亮多于一个当前站。
- **Don't** 在瓷白底上用亮琥珀、亮绿或亮猩红写字。
- **Don't** 只靠颜色区分“已验证”与“未验证”。
- **Don't** 在换页的站点移动之外增加入场动画或滚动动画；现有的动效只有悬停反馈和菜单箭头。
- **Don't** 放入虚构的照片、截图或数字。

## 示意图

正文里的示意图与示意动画是内容的一部分，用同一套色板，但有自己的固定含义：墨色是机械臂，红色（scarlet）是腕部相机及其视野，蓝色（cobalt）是深度相机及其视野，琥珀色（amber）是人的手和舵机电源。关节是白环，与线路图上的站点同一种画法。

- 尺寸取自真实数据：连杆来自项目的运动学，相机位置来自一号套机的外参。
- 画布宽度不超过 480，图里只留短标签，说明放在图下方的图例（`.fg-key`）。
- 动画只表现硬件真实会做的动作，在两个状态之间来回，两头停顿；减少动态效果时停在终态。这不违反“界面动效只有一处”的约定，那条约定管的是导航与控件。
- 样式类以 `fg-` 开头，定义在 `website/src/styles/global.css`；组件在 `website/src/components/figures/`。

