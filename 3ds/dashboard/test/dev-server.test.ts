import { createServer as createHttpServer } from "node:http";
import { once } from "node:events";
import { fileURLToPath } from "node:url";
import { createServer } from "vite";
import { expect, it } from "vitest";

const root = fileURLToPath(new URL("../", import.meta.url));
const configFile = fileURLToPath(new URL("../vite.config.ts", import.meta.url));

it("serves live/mock URLs and transforms the application and ROS entry modules in development mode", async () => {
  const server = await createServer({
    root,
    configFile,
    logLevel: "silent",
    server: { port: 0 },
  });
  try {
    await server.listen();
    const address = server.httpServer!.address();
    if (!address || typeof address === "string")
      throw Error("Missing preview address");
    const base = `http://127.0.0.1:${address.port}`;
    for (const path of [
      "/",
      "/?mode=mock",
      "/?bridge=ws%3A%2F%2Flocalhost%3A9091",
    ]) {
      const response = await fetch(base + path);
      expect(response.status).toBe(200);
      const html = await response.text();
      expect(html).toContain("3DS Drone Monitor");
      expect(html).toContain("/src/main.tsx");
    }
    for (const path of ["/src/main.tsx", "/src/ros.ts"]) {
      const response = await fetch(base + path);
      expect(response.status).toBe(200);
      expect(response.headers.get("content-type")).toContain("javascript");
      const javascript = await response.text();
      expect(javascript.length).toBeGreaterThan(0);
      // Follow optimized imports as a browser would, and let prebundling finish
      // before shutting down the temporary development server.
      for (const [, dependency] of javascript.matchAll(
        /from "(\/node_modules\/\.vite\/deps\/[^"\n]+)"/g,
      )) {
        const module = await fetch(base + dependency);
        expect(module.status).toBe(200);
        expect((await module.text()).length).toBeGreaterThan(0);
      }
    }
    await server.waitForRequestsIdle();
  } finally {
    await server.close();
  }
}, 20000);

it("reports an occupied port instead of silently moving the dashboard to another port", async () => {
  const occupied = createHttpServer();
  occupied.listen(0, "127.0.0.1");
  await once(occupied, "listening");
  const address = occupied.address();
  if (!address || typeof address === "string")
    throw Error("Missing occupied port");
  const server = await createServer({
    root,
    configFile,
    logLevel: "silent",
    server: { port: address.port },
  });
  try {
    await expect(server.listen()).rejects.toThrow(/already in use/);
  } finally {
    await server.close();
    await new Promise<void>((resolve, reject) =>
      occupied.close((error) => (error ? reject(error) : resolve())),
    );
  }
}, 10000);
