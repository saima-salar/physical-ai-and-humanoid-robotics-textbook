import { auth } from "./auth";

const port = 8002;

console.log(`Authentication server running on port ${port}`);

// Export the auth server
export default {
  port,
  ...auth,
};