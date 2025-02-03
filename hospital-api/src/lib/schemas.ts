import { z } from "zod";

export const patientSchema = z.object({
  name: z.string().nonempty("Name is required"),
  address: z.string().nonempty("Address is required"),
  ailment: z.string().nonempty("Ailment is required"),
  facialRecognitionData: z.instanceof(Buffer).optional(),
});
