// TIA Portal V18 Openness bridge for the palletizing simulator.
//
// Input : one JSON line on stdin  {"schema_version":"2.0","operation":"export_project_structure","project_path":"C:\\...\\x.ap18"}
//         (or the project path as the first command-line argument, for manual tests)
// Output: one JSON document on stdout, see docs/openness-contract.json
//
// If the project is already open in a running TIA Portal, the bridge attaches to
// that instance; otherwise it starts TIA Portal without UI, opens the project
// read-only in practice (it is never saved) and closes it at the end.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text;
using System.Web.Script.Serialization;
using Microsoft.Win32;

namespace TiaOpennessBridge
{
    internal static class Program
    {
        private static int Main(string[] args)
        {
            Console.OutputEncoding = new UTF8Encoding(false);
            AppDomain.CurrentDomain.AssemblyResolve += ResolveSiemensAssembly;
            try
            {
                string projectPath = args.Length > 0 ? args[0] : ReadProjectPathFromStdin();
                if (string.IsNullOrWhiteSpace(projectPath))
                {
                    Console.Error.WriteLine("project_path não informado.");
                    return 2;
                }
                projectPath = Path.GetFullPath(projectPath);
                if (!File.Exists(projectPath))
                {
                    Console.Error.WriteLine("Projeto não encontrado: " + projectPath);
                    return 2;
                }
                var response = Exporter.Run(projectPath);
                var serializer = new JavaScriptSerializer { MaxJsonLength = int.MaxValue, RecursionLimit = 512 };
                Console.Out.Write(serializer.Serialize(response));
                Console.Out.Flush();
                return 0;
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine(Describe(ex));
                return 1;
            }
        }

        private static string ReadProjectPathFromStdin()
        {
            string line = Console.In.ReadLine();
            if (string.IsNullOrWhiteSpace(line)) return null;
            var request = new JavaScriptSerializer().Deserialize<Dictionary<string, object>>(line);
            return request.TryGetValue("project_path", out var value) ? value as string : null;
        }

        internal static string Describe(Exception ex)
        {
            var message = new StringBuilder();
            for (var e = ex; e != null; e = e.InnerException)
            {
                if (message.Length > 0) message.Append(" -> ");
                message.Append(e.GetType().Name).Append(": ").Append(e.Message);
            }
            if (ex is FileNotFoundException || ex is FileLoadException || ex is TypeInitializationException)
            {
                message.Append(" | Verifique se o TIA Portal V18 com Openness está instalado e se o usuário pertence ao grupo 'Siemens TIA Openness'.");
            }
            return message.ToString();
        }

        private static IEnumerable<string> SiemensApiDirectories()
        {
            using (var key = Registry.LocalMachine.OpenSubKey(@"SOFTWARE\Siemens\Automation\Openness\18.0\PublicAPI\18.0.0.0"))
            {
                if (key?.GetValue("Siemens.Engineering") is string dll)
                {
                    yield return Path.GetDirectoryName(dll);
                }
            }
            using (var key = Registry.LocalMachine.OpenSubKey(@"SOFTWARE\Siemens\Automation\_InstalledSW\TIAP18\TIA_Opns"))
            {
                if (key?.GetValue("Path") is string path)
                {
                    yield return Path.Combine(path, @"PublicAPI\V18");
                }
            }
            yield return @"C:\Program Files\Siemens\Automation\Portal V18\PublicAPI\V18";
            yield return @"C:\Program Files\Siemens\Automation\Portal V18\Bin\PublicAPI";
        }

        private static Assembly ResolveSiemensAssembly(object sender, ResolveEventArgs args)
        {
            string name = new AssemblyName(args.Name).Name;
            if (!name.StartsWith("Siemens.", StringComparison.OrdinalIgnoreCase)) return null;
            foreach (string dir in SiemensApiDirectories())
            {
                string candidate = Path.Combine(dir, name + ".dll");
                if (File.Exists(candidate)) return Assembly.LoadFrom(candidate);
            }
            return null;
        }
    }

    /// <summary>
    /// Everything touching Siemens.Engineering lives here so the JIT only loads it
    /// after the AssemblyResolve handler is installed.
    /// </summary>
    internal static class Exporter
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static Dictionary<string, object> Run(string projectPath)
        {
            var diagnostics = new List<object>();
            Siemens.Engineering.TiaPortal tia = null;
            Siemens.Engineering.Project project = null;
            bool ownsProject = false;
            string tempDir = Path.Combine(Path.GetTempPath(), "TiaOpennessBridge_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(tempDir);
            try
            {
                foreach (var process in Siemens.Engineering.TiaPortal.GetProcesses())
                {
                    string open = process.ProjectPath?.FullName;
                    if (open != null && string.Equals(Path.GetFullPath(open), projectPath, StringComparison.OrdinalIgnoreCase))
                    {
                        tia = process.Attach();
                        project = tia.Projects.FirstOrDefault(p => string.Equals(p.Path.FullName, projectPath, StringComparison.OrdinalIgnoreCase));
                        diagnostics.Add("Projeto já aberto no TIA Portal (processo " + process.Id + "): usando a instância existente.");
                        break;
                    }
                }
                if (project == null)
                {
                    tia?.Dispose();
                    tia = new Siemens.Engineering.TiaPortal(Siemens.Engineering.TiaPortalMode.WithoutUserInterface);
                    project = tia.Projects.Open(new FileInfo(projectPath));
                    ownsProject = true;
                }

                var plcs = new List<object>();
                foreach (var (deviceName, plc) in FindPlcs(project))
                {
                    plcs.Add(ExportPlc(deviceName, plc, tempDir, ownsProject, diagnostics));
                }
                if (plcs.Count == 0) diagnostics.Add("Nenhuma CPU com software PLC encontrada no projeto.");

                return new Dictionary<string, object>
                {
                    ["project"] = new Dictionary<string, object>
                    {
                        ["name"] = project.Name,
                        ["plcs"] = plcs,
                    },
                    ["diagnostics"] = diagnostics,
                };
            }
            finally
            {
                if (ownsProject)
                {
                    try { project?.Close(); } catch { /* closing without saving */ }
                }
                tia?.Dispose();
                try { Directory.Delete(tempDir, true); } catch { /* temp cleanup is best effort */ }
            }
        }

        private static IEnumerable<(string, Siemens.Engineering.SW.PlcSoftware)> FindPlcs(Siemens.Engineering.Project project)
        {
            var devices = new List<Siemens.Engineering.HW.Device>();
            devices.AddRange(project.Devices);
            foreach (Siemens.Engineering.HW.DeviceUserGroup group in project.DeviceGroups) CollectDevices(group, devices);
            devices.AddRange(project.UngroupedDevicesGroup.Devices);
            var seen = new HashSet<Siemens.Engineering.SW.PlcSoftware>();
            foreach (var device in devices.Distinct())
            {
                foreach (var item in AllItems(device.DeviceItems))
                {
                    var container = item.GetService<Siemens.Engineering.HW.Features.SoftwareContainer>();
                    if (container?.Software is Siemens.Engineering.SW.PlcSoftware plc && seen.Add(plc))
                    {
                        yield return (plc.Name ?? device.Name, plc);
                    }
                }
            }
        }

        private static void CollectDevices(Siemens.Engineering.HW.DeviceUserGroup group, List<Siemens.Engineering.HW.Device> devices)
        {
            devices.AddRange(group.Devices);
            foreach (Siemens.Engineering.HW.DeviceUserGroup child in group.Groups) CollectDevices(child, devices);
        }

        private static IEnumerable<Siemens.Engineering.HW.DeviceItem> AllItems(Siemens.Engineering.HW.DeviceItemComposition items)
        {
            foreach (Siemens.Engineering.HW.DeviceItem item in items)
            {
                yield return item;
                foreach (var child in AllItems(item.DeviceItems)) yield return child;
            }
        }

        private static Dictionary<string, object> ExportPlc(string name, Siemens.Engineering.SW.PlcSoftware plc, string tempDir, bool ownsProject, List<object> diagnostics)
        {
            var tables = new List<object>();
            CollectTagTables(plc.TagTableGroup.TagTables, plc.TagTableGroup.Groups, tables);

            var blocks = new List<object>();
            ExportBlocks(plc.BlockGroup.Blocks, plc.BlockGroup.Groups, tempDir, ownsProject, blocks, diagnostics);

            var types = new List<object>();
            ExportTypes(plc.TypeGroup.Types, plc.TypeGroup.Groups, tempDir, types, diagnostics);

            return new Dictionary<string, object>
            {
                ["name"] = name,
                ["tag_tables"] = tables,
                ["data_blocks"] = blocks,
                ["types"] = types,
            };
        }

        private static void CollectTagTables(
            Siemens.Engineering.SW.Tags.PlcTagTableComposition tagTables,
            Siemens.Engineering.SW.Tags.PlcTagTableUserGroupComposition groups,
            List<object> output)
        {
            foreach (Siemens.Engineering.SW.Tags.PlcTagTable table in tagTables)
            {
                var tags = new List<object>();
                foreach (Siemens.Engineering.SW.Tags.PlcTag tag in table.Tags)
                {
                    tags.Add(new Dictionary<string, object>
                    {
                        ["name"] = tag.Name,
                        ["data_type"] = tag.DataTypeName,
                        ["address"] = tag.LogicalAddress,
                        ["comment"] = FirstText(tag.Comment),
                    });
                }
                output.Add(new Dictionary<string, object> { ["name"] = table.Name, ["tags"] = tags });
            }
            foreach (Siemens.Engineering.SW.Tags.PlcTagTableUserGroup group in groups)
            {
                CollectTagTables(group.TagTables, group.Groups, output);
            }
        }

        private static string FirstText(Siemens.Engineering.MultilingualText text)
        {
            try
            {
                return text?.Items.Select(i => i.Text).FirstOrDefault(t => !string.IsNullOrWhiteSpace(t));
            }
            catch
            {
                return null;
            }
        }

        private static void ExportBlocks(
            Siemens.Engineering.SW.Blocks.PlcBlockComposition blocks,
            Siemens.Engineering.SW.Blocks.PlcBlockUserGroupComposition groups,
            string tempDir,
            bool ownsProject,
            List<object> output,
            List<object> diagnostics)
        {
            foreach (Siemens.Engineering.SW.Blocks.PlcBlock block in blocks)
            {
                if (!(block is Siemens.Engineering.SW.Blocks.DataBlock)) continue;
                var entry = new Dictionary<string, object>
                {
                    ["name"] = block.Name,
                    ["number"] = block.Number,
                };
                try
                {
                    entry["optimized"] = string.Equals(block.GetAttribute("MemoryLayout")?.ToString(), "Optimized", StringComparison.OrdinalIgnoreCase);
                }
                catch
                {
                    // Attribute unavailable (e.g. know-how protected); the XML may still carry it.
                }
                try
                {
                    entry["xml"] = ExportToString(tempDir, "DB_" + block.Number, file => block.Export(file, Siemens.Engineering.ExportOptions.WithDefaults));
                }
                catch (Exception first) when (ownsProject)
                {
                    // Inconsistent blocks cannot be exported: compile in memory (the project is never saved) and retry.
                    try
                    {
                        block.GetService<Siemens.Engineering.Compiler.ICompilable>()?.Compile();
                        entry["xml"] = ExportToString(tempDir, "DB_" + block.Number + "_retry", file => block.Export(file, Siemens.Engineering.ExportOptions.WithDefaults));
                        diagnostics.Add("DB " + block.Name + " estava inconsistente e foi compilado temporariamente para exportação.");
                    }
                    catch (Exception second)
                    {
                        entry["error"] = Program.Describe(first) + " / " + Program.Describe(second);
                    }
                }
                catch (Exception ex)
                {
                    entry["error"] = Program.Describe(ex) + " (compile o DB no TIA Portal e tente novamente)";
                }
                output.Add(entry);
            }
            foreach (Siemens.Engineering.SW.Blocks.PlcBlockUserGroup group in groups)
            {
                ExportBlocks(group.Blocks, group.Groups, tempDir, ownsProject, output, diagnostics);
            }
        }

        private static void ExportTypes(
            Siemens.Engineering.SW.Types.PlcTypeComposition types,
            Siemens.Engineering.SW.Types.PlcTypeUserGroupComposition groups,
            string tempDir,
            List<object> output,
            List<object> diagnostics)
        {
            foreach (Siemens.Engineering.SW.Types.PlcType type in types)
            {
                try
                {
                    output.Add(new Dictionary<string, object>
                    {
                        ["name"] = type.Name,
                        ["xml"] = ExportToString(tempDir, "UDT_" + Guid.NewGuid().ToString("N"), file => type.Export(file, Siemens.Engineering.ExportOptions.WithDefaults)),
                    });
                }
                catch (Exception ex)
                {
                    diagnostics.Add("UDT " + type.Name + ": " + Program.Describe(ex));
                }
            }
            foreach (Siemens.Engineering.SW.Types.PlcTypeUserGroup group in groups)
            {
                ExportTypes(group.Types, group.Groups, tempDir, output, diagnostics);
            }
        }

        private static string ExportToString(string tempDir, string stem, Action<FileInfo> export)
        {
            var file = new FileInfo(Path.Combine(tempDir, stem + ".xml"));
            if (file.Exists) file.Delete();
            export(file);
            return File.ReadAllText(file.FullName, Encoding.UTF8);
        }
    }
}
