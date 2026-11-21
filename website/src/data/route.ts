// 全站的线路。导航、上一站/下一站、首页的线路图都从这里生成。
export type Line = "main" | "site" | "branch" | "ref";

export interface Station {
  path: string; // 站内路径，首页是 "/"
  name: string; // 站名
  note: string; // 一句话说明，显示在首页线路表里
  line: Line;
}

export const LINES: Record<Line, { name: string; who: string }> = {
  main: { name: "主线", who: "现有套机，换了一台电脑" },
  site: { name: "现场线", who: "跑起来之后，在路演现场" },
  branch: { name: "新套机支线", who: "同型号的全新套机" },
  ref: { name: "速查线", who: "随时查阅" },
};

export const MAIN: Station[] = [
  { path: "/", name: "总览", note: "这是什么，走哪条线", line: "main" },
  { path: "/install/", name: "安装", note: "装依赖，不接硬件先跑通", line: "main" },
  { path: "/power/", name: "接线与上电", note: "三块控制板、供电、确认应答", line: "main" },
  { path: "/config/", name: "本机配置", note: "套机档案、腕部相机、云台", line: "main" },
  { path: "/first-run/", name: "首次启动", note: "逐项验证，手放在空格键上", line: "main" },
  { path: "/show/", name: "路演值守", note: "常驻运行、来人恢复、看门狗", line: "site" },
  { path: "/troubleshoot/", name: "排障", note: "现象与处理", line: "site" },
];

// 支线在“接线与上电”离开主线，在“首次启动”回到主线
export const BRANCH: Station[] = [
  { path: "/new-kit/", name: "新套机上电", note: "分辨控制板、核对校准、云台零点", line: "branch" },
  { path: "/calibration/", name: "外参标定", note: "让深度相机知道两只臂在哪", line: "branch" },
];
export const BRANCH_FROM = "/power/";
export const BRANCH_TO = "/first-run/";

export const REF: Station[] = [
  { path: "/reference/", name: "速查", note: "按键、命令、配置分层", line: "ref" },
];

export const ALL: Station[] = [...MAIN, ...BRANCH, ...REF];

export function find(path: string): Station | undefined {
  const norm = path.endsWith("/") ? path : path + "/";
  return ALL.find((s) => s.path === norm);
}

/** 上一站与下一站。支线两端接回主线上的换乘站。 */
export function neighbours(path: string): { prev?: Station; next?: Station } {
  const here = find(path);
  if (!here) return {};
  if (here.line === "main" || here.line === "site") {
    const i = MAIN.indexOf(here);
    return { prev: MAIN[i - 1], next: MAIN[i + 1] };
  }
  if (here.line === "branch") {
    const i = BRANCH.indexOf(here);
    return {
      prev: BRANCH[i - 1] ?? find(BRANCH_FROM),
      next: BRANCH[i + 1] ?? find(BRANCH_TO),
    };
  }
  return { prev: find("/") };
}
