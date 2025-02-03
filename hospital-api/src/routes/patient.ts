import { createPatientHandler } from "../controllers/patient";

export const patientRoutes = {
  POST: {
    "/patients": createPatientHandler,
  },
  GET: {},
  PUT: {},
  DELETE: {},
};
