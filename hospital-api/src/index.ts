import { serve } from "bun";
import { routes } from "./routes";

const server = serve({
  async fetch(req: Request) {
    const url = new URL(req.url);
    const method = req.method as keyof typeof routes;
    const route = routes[method]?.[url.pathname];

    if (route) {
      return await route(req);
    } else {
      return new Response("Not Found", { status: 404 });
    }
  },
  port: 8080,
});

console.log(server.url);
