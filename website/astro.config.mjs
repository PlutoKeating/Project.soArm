import { defineConfig } from "astro/config";
import mdx from "@astrojs/mdx";

// 纯静态输出到 dist/，由 Cloudflare Workers 的静态资源托管（见 wrangler.jsonc）
export default defineConfig({
  integrations: [mdx()],
  trailingSlash: "always",
  build: { format: "directory" },
  markdown: {
    // 代码配色交给 CSS 变量，和站点的色板放在一处（src/styles/global.css）
    shikiConfig: { theme: "css-variables", wrap: false },
  },
});
