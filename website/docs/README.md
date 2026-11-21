# 文档站点

把仓库里的上手文档转述成一个按步骤走的站点。读者是新加入的同学和接手路演的同事。

技术栈：[Astro](https://astro.build/) 生成纯静态页面，内容用 MDX 写，字体自托管，部署到 Cloudflare Workers。

## 本地开发

需要 Node.js 22.23.4 或更新的版本。

```bash
cd website
npm install
npm run dev        # http://localhost:4321，改文件即时刷新
npm run build      # 构建到 dist/
npm run preview    # 预览构建结果
```

## 目录

```text
src/data/route.ts          全站线路：有哪些站、属于哪条线、支线从哪离开又回到哪
src/layouts/Doc.astro      页面骨架：线路牌、正文、上一站 / 下一站
src/components/Rail.astro  线路图导航
src/components/Notice.astro    通告（机械臂会动 / 注意 / 说明 / 尚未在真机验证）
src/components/Mark.astro      验证状态的记号
src/components/RouteTable.astro  首页的线路表
src/components/KitFigure.astro   套机俯视示意图
src/pages/*.mdx            每一站一个文件
src/styles/global.css      全部样式和色板
public/                    原样拷贝的静态文件
wrangler.jsonc             Cloudflare Workers 的部署配置
```

## 写内容

- **改一站的内容**：编辑 `src/pages/<站>.mdx`。二级标题会自动编号成步骤；不是步骤的页面在文件头写 `steps: false`。
- **加一站**：新建 `src/pages/<路径>.mdx`，再在 `src/data/route.ts` 里把它加到对应的线上。导航、上一站 / 下一站、首页线路表都由这个文件生成。
- **提示**：`<Notice kind="moves">`（机械臂会动）、`caution`、`info`、`unverified`。凡是会让硬件动的步骤，前面都放一个 `moves`。
- 内容以仓库根目录的 `README.md` 和 `docs/` 为准。命令或流程变了，两边一起改。
- 没在真机上验证过的东西用 `unverified` 标出来，不要写成已经验证。

## 部署到 Cloudflare Workers

站点只有静态文件，没有 Worker 脚本。`wrangler.jsonc` 指定把 `dist/` 作为静态资源托管。

在 Cloudflare 控制台新建 Worker 并连接 Git 仓库（Workers Builds）时这样填：

| 字段 | 填什么 |
|---|---|
| Worker 名称 | `soarm-docs`，必须与 `wrangler.jsonc` 里的 `name` 一致，否则构建失败。想换名字就两边一起改 |
| Git repository | 本仓库 |
| Git branch（生产分支） | `main` |
| Root directory | `website` |
| Build command | `npm ci && npm run build` |
| Deploy command | `npx wrangler deploy`（默认值） |
| Preview command（非生产分支） | 保持默认 |
| Build variables | 不需要 |

建议在 Settings → Build → Build watch paths 里把 include 设成 `website/*`，这样只改机器人代码时不会触发站点构建。

`.node-version` 写的是 22。Cloudflare 的构建环境是否读取这个文件没有核实过；如果构建时报 Node 版本过低，在 Build variables 里加 `NODE_VERSION`，值为 `22`。构建命令里写了 `npm ci`，是因为没有核实构建环境是否会自动安装依赖，多装一次没有坏处。

也可以从本机手动发布：`npm run deploy`（首次会要求登录 Cloudflare）。

设计上的约定见 [ARCHITECTURE.md](ARCHITECTURE.md)。
