import { db } from "../lib/prisma";
import { patientSchema } from "../lib/schemas";

export async function createPatient(patientData: unknown) {
  const validatedPatient = patientSchema.parse(patientData);
  const newPatient = await db.patient.create({
    data: validatedPatient,
  });

  return newPatient;
}
