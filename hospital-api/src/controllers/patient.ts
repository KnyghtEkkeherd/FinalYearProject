import { createPatient } from "../repository/patient";

const isError = (error: unknown): error is Error => {
  return error instanceof Error;
};

export const createPatientHandler = async (req: Request): Promise<Response> => {
  try {
    const patientData = await req.json();
    const newPatient = await createPatient(patientData);
    return new Response(JSON.stringify(newPatient), {
      status: 201,
      headers: { "Content-Type": "application/json" },
    });
  } catch (error) {
    if (isError(error)) {
      return new Response(JSON.stringify({ message: error.message }), {
        status: 400,
        headers: { "Content-Type": "application/json" },
      });
    }
    return new Response(JSON.stringify({ message: "Unknown error occurred" }), {
      status: 400,
      headers: { "Content-Type": "application/json" },
    });
  }
};
