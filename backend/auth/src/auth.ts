import { betterAuth } from "better-auth";

export const auth = betterAuth({
  secret: process.env.AUTH_SECRET || "vryLodo6K5HvQj7P9dY2gF3nR8sW1mE5tH4cB7aZ6xJ3vN9oL2pQ5sM8wE1rT4yU7iO",
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  socialProviders: {
    // Add social providers if needed
  },
  user: {
    additionalFields: {
      // Store user background information for personalization
      softwareExperience: {
        type: "string",
        required: false,
      },
      hardwareExperience: {
        type: "string",
        required: false,
      },
      programmingLanguages: {
        type: "string", // JSON string
        required: false,
      },
      roboticsExperience: {
        type: "string",
        required: false,
      },
      learningGoals: {
        type: "string", // JSON string
        required: false,
      },
      skillLevel: {
        type: "string",
        required: false,
      },
      domainInterest: {
        type: "string",
        required: false,
      }
    }
  }
});