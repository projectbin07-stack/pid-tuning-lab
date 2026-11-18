#!/usr/bin/env node
import { writeFileSync } from "node:fs";
import { main } from "../src/cli.js";

process.exitCode = main(process.argv.slice(2), {
  out: (text) => process.stdout.write(text),
  err: (text) => process.stderr.write(text),
  writeFile: (path, data) => writeFileSync(path, data),
});
