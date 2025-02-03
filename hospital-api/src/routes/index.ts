import { patientRoutes } from "./patient";

type RouteHandlers = {
  [key: string]: (req: Request) => Promise<Response>;
};

type HttpMethods = {
  GET: RouteHandlers;
  POST: RouteHandlers;
  PUT: RouteHandlers;
  DELETE: RouteHandlers;
};

export const routes: HttpMethods = {
  GET: {},
  POST: {
    ...patientRoutes.POST,
  },
  PUT: {},
  DELETE: {},
};
