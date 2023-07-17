use core::str::FromStr;

use proc_macro2::TokenStream;
use syn::DeriveInput;

fn sync_comp_group_impl(ast : DeriveInput) -> proc_macro::TokenStream {
    match ast.data {
        syn::Data::Struct(data) => {
            let name = ast.ident;
            let fields = data.fields;
            let fields_count = fields.len();

            let mut setup_stream = TokenStream::new();
            let mut index_stream = TokenStream::new();
            let mut index_stream_mut = TokenStream::new();

            let mut f_index : usize  = 0;
            for field in fields {
                let field_name = field.ident;
                let field_stream = if let Some(n) = field_name.clone() {
                    quote::quote! { &self.#n }
                } else {
                    TokenStream::from_str(&format!("&self.{}", f_index)).unwrap()
                };

                index_stream.extend::<TokenStream>(
                    quote::quote! {
                        if index == #f_index {
                            return #field_stream;
                        }
                    }.into()
                );

                let field_stream = if let Some(n) = field_name.clone() {
                    quote::quote! { &mut self.#n }
                } else {
                    TokenStream::from_str(&format!("&mut self.{}", f_index)).unwrap()
                };

                index_stream_mut.extend::<TokenStream>(
                    quote::quote! {
                        if index == #f_index {
                            return #field_stream;
                        }
                    }.into()
                );

                let field_stream = if let Some(n) = field_name.clone() {
                    quote::quote! { self.#n }
                } else {
                    TokenStream::from_str(&format!("self.{}", f_index)).unwrap()
                };

                setup_stream.extend::<TokenStream>(
                    quote::quote! {
                        #field_stream.setup()?;
                    }.into()
                );

                f_index += 1;
            }

            index_stream.extend::<TokenStream>(
                quote::quote! {
                    panic!("Index {} is out of bounds", index)
                }.into()
            );

            index_stream_mut.extend::<TokenStream>(
                quote::quote! {
                    panic!("Index {} is out of bounds", index)
                }.into()
            );

            quote::quote! {
                impl Setup for #name {
                    fn setup(&mut self) -> Result<(), syact::Error> {
                        #setup_stream
                        Ok(())
                    }
                }

                impl SyncCompGroup<#fields_count> for #name { 
                    fn index(&self, index : usize) -> &dyn SyncComp {
                        #index_stream
                    }

                    fn index_mut(&mut self, index : usize) -> &mut dyn SyncComp {
                        #index_stream_mut
                    }
                }
            }.into()
        },
        _ => panic!("This macro can only be used on structs")
    }
}

#[proc_macro_derive(SyncCompGroup)]
pub fn sync_comp_group_derive(input : proc_macro::TokenStream) -> proc_macro::TokenStream {
    let ast : DeriveInput = syn::parse(input).unwrap();
    sync_comp_group_impl(ast)
}

fn stepper_comp_group_impl(ast : DeriveInput) -> proc_macro::TokenStream {
    match ast.data {
        syn::Data::Struct(data) => {
            let name = ast.ident;
            let fields = data.fields;
            let fields_count = fields.len();

            let mut builder_stream = TokenStream::new();
            let mut drive_stream = TokenStream::new();
            let mut micro_stream = TokenStream::new();

            let mut f_index : usize  = 0;
            for field in fields {
                let field_name = field.ident;
                
                builder_stream.extend::<TokenStream>(if let Some(n) = field_name.clone() {
                    quote::quote! { self.#n.create_curve_builder(omega_0[#f_index]), }
                } else {
                    TokenStream::from_str(&format!("self.{}.create_curve_builder(omega_0[{}]),", f_index, f_index)).unwrap()
                });

                drive_stream.extend::<TokenStream>(if let Some(n) = field_name.clone() {
                    quote::quote! { self.#n.drive_nodes(nodes_0[#f_index].delta, nodes_0[#f_index].omega_0, omega_tar[#f_index], &mut corr[#f_index])?; }
                } else {
                    TokenStream::from_str(&format!("self.{}.drive_nodes(nodes_0[{}].delta, nodes_0[{}].omega_0, omega_tar[{}], &mut corr[{}])?;", 
                        f_index, f_index, f_index, f_index, f_index)).unwrap()
                });

                micro_stream.extend::<TokenStream>(if let Some(n) = field_name.clone() {
                    quote::quote! { self.#n.set_micro(micro[#f_index]); }
                } else {
                    TokenStream::from_str(&format!("self.{}.set_micro(micro[{}]);", f_index, f_index)).unwrap()
                });

                f_index += 1;
            }

            quote::quote! {
                impl syact::comp::stepper::StepperCompGroup<#fields_count> for #name { 
                    fn create_path_builder(&self, omega_0 : [syact::units::Omega; #fields_count]) 
                    -> syact::math::path::PathBuilder<#fields_count> {
                        PathBuilder::new([
                            #builder_stream
                        ])
                    }

                    fn drive_nodes(&mut self, 
                        nodes_0 : &[syact::math::path::PathNode; #fields_count], 
                        omega_tar : [syact::units::Omega; #fields_count], 
                        corr : &mut [(syact::units::Delta, syact::units::Time); #fields_count],
                    ) -> Result<(), syact::Error> {
                        #drive_stream
                        Ok(())
                    }

                    fn set_micro(&mut self, micro : [u8; #fields_count]) {
                        #micro_stream
                    }
                }
            }.into()
        },
        _ => panic!("This macro can only be used on structs")
    }
}

#[proc_macro_derive(StepperCompGroup)]
pub fn stepper_comp_group_derive(input : proc_macro::TokenStream) -> proc_macro::TokenStream {
    let ast : DeriveInput = syn::parse(input).unwrap();
    stepper_comp_group_impl(ast)
}