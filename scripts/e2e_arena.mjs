/* E2E: the arena in a real browser, headless. Serves site/ statically,
   boots arena.html (rapier WASM from the CDN import map, exactly like a
   player), and drives the robot with WASD — asserting the physics moves
   the body, walls stop it, and no page errors fire on boot.

   Run:
     python scripts/build_site.py
     node scripts/e2e_arena.mjs        # needs a cached playwright chromium
   Expected: E2E ARENA OK */
import { createServer } from "http";
import { readFileSync, readdirSync, existsSync } from "fs";
import { execSync } from "child_process";
import { homedir } from "os";
import { join } from "path";
import { chromium } from "playwright-core";

const SITE = new URL("../site", import.meta.url).pathname;
const MIME = { html: "text/html", js: "text/javascript", css: "text/css",
               json: "application/json", py: "text/x-python" };

function chromePath() {
  const env = process.env.CHROMIUM_PATH;
  if (env) return env;
  const cache = join(homedir(), "Library/Caches/ms-playwright");
  const shells = readdirSync(cache).filter((d) => d.startsWith("chromium_headless_shell-")).sort();
  for (const shell of shells.reverse()) {
    const hit = execSync(`find "${join(cache, shell)}" -name chrome-headless-shell -type f`,
                         { encoding: "utf8" }).trim().split("\n")[0];
    if (hit && existsSync(hit)) return hit;
  }
  throw new Error("no cached chromium headless shell; set CHROMIUM_PATH");
}

const server = createServer((req, res) => {
  const path = join(SITE, req.url === "/" ? "arena.html" : req.url.split("?")[0]);
  try {
    const body = readFileSync(path);
    const ext = path.split(".").pop();
    res.writeHead(200, { "Content-Type": MIME[ext] || "application/octet-stream" });
    res.end(body);
  } catch { res.writeHead(404); res.end(); }
});
await new Promise((r) => server.listen(0, r));
const port = server.address().port;

let failures = 0;
const check = (name, cond, detail = "") => {
  console.log(`${cond ? "ok " : "FAIL"} ${name}${detail ? " — " + detail : ""}`);
  if (!cond) failures++;
};

const browser = await chromium.launch({ executablePath: chromePath() });
const page = await browser.newPage();
const pageErrors = [];
page.on("pageerror", (e) => pageErrors.push(String(e)));

await page.goto(`http://127.0.0.1:${port}/arena.html`, { waitUntil: "load" });
// boot = rapier init + level build; the brief title is set by loadLevel(0)
await page.waitForFunction(
  () => document.getElementById("briefTitle")?.textContent?.includes("DOG"),
  null, { timeout: 30000 });
check("arena boots (rapier + level 0)", true);

await page.click("#startClose");                  // dismiss the start screen

const pose = async () => {
  const t = await page.textContent("#telePose");
  const m = t.match(/x (-?[\d.]+) · z (-?[\d.]+)/);
  return m ? { x: +m[1], z: +m[2] } : null;
};

const p0 = await pose();
await page.keyboard.down("w");
await page.waitForTimeout(1500);
await page.keyboard.up("w");
const p1 = await pose();
check("WASD drives the body through physics",
      p0 && p1 && Math.hypot(p1.x - p0.x, p1.z - p0.z) > 0.5,
      `moved ${p0 && p1 ? Math.hypot(p1.x - p0.x, p1.z - p0.z).toFixed(2) : "?"}m`);

// drive into the arena boundary for a while — the wall must hold
await page.keyboard.down("w");
await page.waitForTimeout(9000);
await page.keyboard.up("w");
const p2 = await pose();
check("walls contain the robot", p2 && Math.abs(p2.x) < 16 && Math.abs(p2.z) < 16,
      `x=${p2?.x} z=${p2?.z}`);

const odo = await page.textContent("#teleOdo");
check("odometer counts", parseFloat(odo.replace(/[^\d.]/g, "")) > 1, odo);

const fatal = pageErrors.filter((e) => !/pyodide|wasm\.js|fetch/i.test(e));
check("no page errors", fatal.length === 0, fatal.slice(0, 2).join(" | "));

await browser.close();
server.close();
console.log(failures ? `E2E ARENA FAILED (${failures})` : "E2E ARENA OK");
process.exit(failures ? 1 : 0);
